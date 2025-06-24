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

        [SerializeField]
        private Transform follower;

        private void Update()
        {
            if (target != null)
            {
                var pose = new Pose3();
                pose.X = target.position.x;
                pose.Y = target.position.y;
                pose.Z = target.position.z;

                var gens = controller.SetInverse(pose);
                var forwardSolution = controller.GetForward(gens);
                follower.position = new Vector3(forwardSolution.X, forwardSolution.Y, forwardSolution.Z);
            }
        }
    }
}