import torch
import torch.nn as nn
import torch.nn.functional as F

import torch_geometric.nn as pyg_nn
import torch_geometric.utils as pyg_utils

class GNNStack(torch.nn.Module):
    def __init__(self, input_dim, hidden_dim, output_dim, args, task='node'):
        super(GNNStack, self).__init__()
        self.task = task
        self.convs = nn.ModuleList()
        self.convs.append(self.build_conv_model(args.model_type, input_dim, hidden_dim))
        assert (args.num_layers >= 1), 'Number of layers is not >=1'
        for l in range(args.num_layers-1):
            self.convs.append(self.build_conv_model(args.model_type, hidden_dim, hidden_dim))

        # post-message-passing
        # Sequential : will be called sequentially
        self.post_mp = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim), nn.Dropout(args.dropout), 
            nn.Linear(hidden_dim, output_dim))

        # check undefined task
        if not (self.task == 'node' or self.task == 'graph'):
            raise RuntimeError('Unknown task.')

        self.dropout = args.dropout
        self.num_layers = args.num_layers

    def build_conv_model(self, model_type, input_dim, hidden_dim):
        if True:
            # use a simple GCN for node embedding
            if model_type == 'GCN':
                return pyg_nn.GCNConv(input_dim, hidden_dim)
            elif model_type == 'GraphSage':
                return GraphSage(input_dim, hidden_dim)
            elif model_type == 'GAT':
                return GAT(input_dim, hidden_dim)
        else:
            # for whole graph embedding
            return pyg_nn.GINConv(nn.Sequential(nn.Linear(input_dim, hidden_dim),
                                  nn.ReLU(), nn.Linear(hidden_dim, hidden_dim)))
    

    def forward(self, data):
        x, edge_index, batch = data.x, data.edge_index, data.batch

        ############################################################################
        # TODO: Your code here! 
        # Each layer in GNN should consist of a convolution (specified in model_type),
        # a non-linearity (use RELU), and dropout. 
        # HINT: the __init__ function contains parameters you will need. You may 
        # also find pyg_nn.global_max_pool useful for graph classification.
        # Our implementation is ~6 lines, but don't worry if you deviate from this.

        # sanity check
        if data.num_node_features == 0:
            x = torch.ones(data.num_nodes, 1)

        for i in range(self.num_layers):
            x = self.convs[i](x, edge_index)
            emb = x
            x = F.relu(x)
            x = F.dropout(x, p=self.dropout, training=self.training)

        # global_mean_pool will lead to a terrible result
        if self.task == 'graph':
            x = pyg_nn.global_max_pool(x, batch)

        ############################################################################

        x = self.post_mp(x)

        return F.log_softmax(x, dim=1)

    def loss(self, pred, label):
        return F.nll_loss(pred, label)

class GraphSage(pyg_nn.MessagePassing):
    """Non-minibatch version of GraphSage."""
    def __init__(self, in_channels, out_channels, reducer='mean', 
                 normalize_embedding=True):
        super(GraphSage, self).__init__(aggr='mean') # /space 

        self.lin = nn.Linear(in_channels, in_channels)
        self.agg_lin = nn.Linear(in_channels+in_channels, out_channels)

        self.normailze_agg = False
        if normalize_embedding:
            self.normalize_emb = True

    def forward(self, x, edge_index):
        # remove the self edges, as we will concate the self features in the update stage.
        edge_index, _ = pyg_utils.remove_self_loops(edge_index)
        return self.propagate(edge_index, x=x)

    def message(self, x_j, edge_index):
        if(self.normailze_agg):
            row, col = edge_index
            deg = pyg_utils.degree(col, x_j.size(0), dtype=x_j.dtype)
            deg_inv_sqrt = deg.pow(-0.5)
            norm = deg_inv_sqrt[row] * deg_inv_sqrt[col]
            return norm.view(-1, 1) * x_j
        else :
            return x_j

    def update(self, aggr_out, x):
        concat_out = torch.cat((x, aggr_out), 1)
        aggr_out = F.relu(self.agg_lin(concat_out)) 
        if self.normalize_emb:
            aggr_out = F.normalize(aggr_out, p=2, dim=1) 
        return aggr_out


class GAT(pyg_nn.MessagePassing):

    def __init__(self, in_channels, out_channels, num_heads=1, concat=True,
                 dropout=0, bias=True, **kwargs):
        super(GAT, self).__init__(aggr='add', **kwargs)

        self.in_channels = in_channels
        self.out_channels = out_channels
        self.heads = num_heads
        self.concat = concat 
        self.dropout = dropout

        ############################################################################
        #  TODO: Your code here!
        # Define the layers needed for the forward function. 
        # Remember that the shape of the output depends the number of heads.
        # Our implementation is ~1 line, but don't worry if you deviate from this.

        self.lin = nn.Linear(self.in_channels, self.out_channels * self.heads)

        ############################################################################

        ############################################################################
        #  TODO: Your code here!
        # The attention mechanism is a single feed-forward neural network parametrized
        # by weight vector self.att. Define the nn.Parameter needed for the attention
        # mechanism here. Remember to consider number of heads for dimension!
        # Our implementation is ~1 line, but don't worry if you deviate from this.

        self.att = nn.Parameter(torch.Tensor(1, self.heads, self.out_channels * 2))

        ############################################################################

        if bias and concat:
            self.bias = nn.Parameter(torch.Tensor(self.heads * self.out_channels))
        elif bias and not concat:
            self.bias = nn.Parameter(torch.Tensor(out_channels))
        else:
            self.register_parameter('bias', None)

        nn.init.xavier_uniform_(self.att)
        nn.init.zeros_(self.bias)

        ############################################################################

    def forward(self, x, edge_index, size=None):
        ############################################################################
        #  TODO: Your code here!
        # Apply your linear transformation to the node feature matrix before starting
        # to propagate messages.
        # Our implementation is ~1 line, but don't worry if you deviate from this.
        
        x = self.lin(x) 
        ############################################################################

        # Start propagating messages.
        return self.propagate(edge_index, size=size, x=x)

    def message(self, edge_index_i, x_i, x_j, size_i):
        #  Constructs messages to node i for each edge (j, i).

        ############################################################################
        #  TODO: Your code here! Compute the attention coefficients alpha as described
        # in equation (7). Remember to be careful of the number of heads with 
        # dimension!
        # Our implementation is ~5 lines, but don't worry if you deviate from this.
        [shape0, shape1] = x_j.shape
        x_i = x_i.view(-1, self.heads, self.out_channels)
        x_j = x_j.view(-1, self.heads, self.out_channels)
        alpha = (torch.cat([x_i, x_j], dim=-1) * self.att).sum(dim=-1)
        alpha = F.leaky_relu(alpha, 0.2)
        alpha = pyg_utils.softmax(alpha, edge_index_i)

        ############################################################################

        alpha = F.dropout(alpha, p=self.dropout, training=self.training)
        out = (x_j * alpha.view(-1, self.heads,1)).view(shape0, shape1);
        return out

    def update(self, aggr_out):
        # Updates node embedings.
        if self.concat is True:
            aggr_out = aggr_out.view(-1, self.heads * self.out_channels)
        else:
            aggr_out = aggr_out.mean(dim=1)

        if self.bias is not None:
            aggr_out = aggr_out + self.bias
        return aggr_out
