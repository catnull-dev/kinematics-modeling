using KinematicsModeling.Robots.RRR;
using UnityEngine;

namespace KinematicsModeling.Target
{
    public class Target : MonoBehaviour
    {
        [SerializeField]
        private Controller controller;

        [SerializeField]
        private Transform target;

        private void Update()
        {
            if (target != null)
            {
                var pose = new Pose3();
                pose.X = target.position.x;
                pose.Y = target.position.y;
                pose.Z = target.position.z;

                controller.SetInverse(pose);
            }
        }
    }
}