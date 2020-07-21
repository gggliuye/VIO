#include "ScaleEstimator.h"
#include "AnalysisTool.h"

/*
./AnalysisMap /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/database.db /home/viki/UTOPA/Server_Localization/Maps/winter_garden_final/dense/sparse /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words32K.bin 2.81


./AnalysisMap /home/viki/Lucas/garden/garden_v2/database.db /home/viki/Lucas/garden/garden_v2/sparse/  /home/viki/UTOPA/Server_Localization/vocabs/vocab_tree_flickr100K_words32K.bin 3.837999
*/


int main(int argc, char** argv) {
    if(argc != 5)
    {
        std::cerr << std::endl << "Usage: ./AnalysisMap database_path sparse_map_path voc_indices_path re_scale\n";
        return 1;
    }

    Ulocal::LocalizationLY *pLocalizationLY;
    pLocalizationLY = new Ulocal::LocalizationLY(argv[1], argv[2], argv[3], true, false);

    Ulocal::AnalysisTool *pAnalysisTool = new Ulocal::AnalysisTool(pLocalizationLY);

    float scale = atof(argv[4]);

    cv::Mat image_show;
    Eigen::Matrix4d projectionToPlane;
    projectionToPlane.setIdentity();
    Eigen::Vector2d vHeightRange(-3,2);
    pAnalysisTool->ProjectMapIntoImage(image_show, projectionToPlane, vHeightRange, scale, true);
    pAnalysisTool->TestMapRead(pAnalysisTool->output_file);

    cv::imwrite("analysis_show.png", image_show);

    //pLocalizationLY->View();

}
