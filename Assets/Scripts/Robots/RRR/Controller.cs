using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using KinematicsModeling;
using System;

namespace KinematicsModeling.Robots.RRR
{
    [ExecuteAlways]
    public class Controller : MonoBehaviour, IController<Gen3, Pose3>
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

        [SerializeField]
        private bool config1;

        private Flag flag;

        private IInverseKinematicsSolver<Pose3, Gen3, Flag, RobotConfiguration3> inverseSolver;

        private IForwardKinematicsSolver<Gen3, Pose3> forwardSolver;

        private RobotConfiguration3 robotConfiguration;

        private void Start()
        {
            robotConfiguration = new RobotConfiguration3();
            robotConfiguration.LengthLink1 = 0.557f;
            robotConfiguration.LengthLink2 = 0.501f;
            robotConfiguration.LengthLink3 = 0.501f;

            flag = new Flag();
            flag.Value = 1;

            inverseSolver = new InverseSolver();
        }

        public Pose3 SetForward(Gen3 gens)
        {
            if (forwardSolver == null)
            {
                throw new NullReferenceException("Add " + nameof(forwardSolver) +  " to controller.");
            }

            var result = forwardSolver.SolveForwardKinematics(gens);
            joint1 = gens.Joint1;
            joint2 = gens.Joint2;
            joint3 = gens.Joint3;

            return result;
        }

        public Gen3 SetInverse(Pose3 pose)
        {
            if (inverseSolver == null)
            {
                throw new NullReferenceException("Add " + nameof(inverseSolver) + " to controller.");
            }

            var result = inverseSolver.SolveInverseKinematics(pose, flag, robotConfiguration);

            joint1 = result.Joint1;
            joint2 = result.Joint2;
            joint3 = result.Joint3;

            return result;
        }

        private void Update()
        {
            robotBase.localRotation = Quaternion.AngleAxis(joint1, new Vector3(0, 1, 0));
            link1.localRotation = Quaternion.AngleAxis(joint2, new Vector3(0, 0, 1));
            link2.localRotation = Quaternion.AngleAxis(joint3, new Vector3(0, 0, 1));

            if (config1)
            {
                flag.Value = 1;
            }
            else
            {
                flag.Value = -1;
            }
        }
    }

    internal interface IController<Gen, Pose>
        where Gen: struct
        where Pose: struct
    {
        Pose SetForward(Gen gens);
        Gen SetInverse(Pose pose);
    }

    internal class InverseSolver : IInverseKinematicsSolver<Pose3, Gen3, Flag, RobotConfiguration3>
    {
        public Gen3 SolveInverseKinematics(Pose3 target, Flag flag, RobotConfiguration3 configuration)
        {
            var translation = new Vector3(target.X, target.Y, target.Z);

            var q1 = Mathf.Atan2(-target.Z, target.X);

            var r = Mathf.Sqrt(target.Z * target.Z + target.X * target.X);

            var b = target.Y - configuration.LengthLink1;
            var c = Mathf.Sqrt(b * b + target.X * target.X);

            var omega = Mathf.Atan2(b, r);
            var gammaPart = Mathf.Pow(configuration.LengthLink2, 2) + c * c - Mathf.Pow(configuration.LengthLink3, 2);
            var gamma = Mathf.Acos(gammaPart / (2 * c * configuration.LengthLink2));

            var q2 = omega - gamma;

            var sigmaPart = Mathf.Pow(configuration.LengthLink2, 2) + Mathf.Pow(configuration.LengthLink3, 2) - c * c;
            var sigma = Mathf.Acos(sigmaPart / (2 * configuration.LengthLink2 * configuration.LengthLink3));
            var q3 = Mathf.PI - sigma;

            var result = new Gen3();
            result.Joint1 = Mathf.Rad2Deg * q1;
            result.Joint2 = Mathf.Rad2Deg * q2;
            result.Joint3 = Mathf.Rad2Deg * q3;

            return result;
        }
    }
}