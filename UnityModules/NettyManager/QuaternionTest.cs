using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteInEditMode]
public class QuaternionTest : MonoBehaviour {

    private IEnumerator coroutine;
    private Quaternion quaternion;

	// Use this for initialization
	void Start () {
        quaternion = Quaternion.identity;
    }
	
	// Update is called once per frame
	void Update () {
        this.transform.rotation = quaternion;
        //transform.rotation = Quaternion.Lerp(from.rotation, to.rotation, Time.time * speed);
    }

    /*
     * Coroutines only work in Play mode. The yield functions work according
     * to the in-game time and frames and can't be used outside of this context.
     */
    public void StartRotate()
    {
        coroutine = WaitAndRotate(1.0f);
        StartCoroutine(coroutine);
    }

    public void Test()
    {
        if (coroutine != null)
        {
            print("StopCoroutine " + Time.time);
            StopCoroutine(coroutine);
        }
        else
        {
            print("Coroutine is null.");
        }
    }

    private IEnumerator WaitAndRotate(float waitTime)
    {
        for(int i = 0; i < 10; i++)
        {
            quaternion = quaternion * Quaternion.Euler(1, 1, 1);
            yield return new WaitForSeconds(waitTime);
            print("WaitAndPrint " + Time.time + " i:" + i);
        }
    }

}
