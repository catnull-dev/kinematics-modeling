using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[ExecuteAlways]
public class Controller : MonoBehaviour
{
    [SerializeField]
    private float joint1;

    [SerializeField]
    private float joint2;

    [SerializeField]
    private float joint3;

    [SerializeField]
    private Transform robotBase;

    [SerializeField]
    private Transform link1;

    [SerializeField]
    private Transform link2;

    private void Update()
    {
        robotBase.localRotation = Quaternion.AngleAxis(joint1, new Vector3(0, 1, 0));
        link1.localRotation = Quaternion.AngleAxis(joint2, new Vector3(0, 0, 1));
        link2.localRotation = Quaternion.AngleAxis(joint3, new Vector3(0, 0, 1));
    }
}
