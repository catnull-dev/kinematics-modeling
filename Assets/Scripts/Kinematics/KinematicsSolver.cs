namespace KinematicsModeling
{
    public interface IInverseKinematicsSolver<Target, Output, Flags, Configuration>
        where Target : struct
        where Output : struct
        where Flags : struct
        where Configuration : struct
    {
        Output SolveInverseKinematics(Target target, Flags flags, Configuration configuration);
    }

    public interface IForwardKinematicsSolver<Target, Output>
        where Output : struct
        where Target : struct
    {
        Output SolveForwardKinematics(Target target);
    }

    public struct Pose3 {
        public float X { get; set; }
        public float Y { get; set; }
        public float Z { get; set; }
    }

    public struct Gen3
    {
        public float Joint1 { get; set; }
        public float Joint2 { get; set; }
        public float Joint3 { get; set; }
    }

    public struct Flag
    {
        public int Value { get; set; }
    }

    public struct RobotConfiguration3
    {
        public float LengthLink1 { get; set; }
        public float LengthLink2 { get; set; }
        public float LengthLink3 { get; set; }
    }
}