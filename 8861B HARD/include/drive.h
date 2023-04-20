

class Drive{
  public:
  static void RobotOriented();
  static void SetVoltage(double Left, double right);
  static void expansion();
  static void Intake();
  static void Shoot();
  static void Turn(int x);
  static void Move(int x, int spd);
  static void Shots(int spd, int deg, int indexdeg);
  static void Inroll(int x);
  static void flappy();
  static int updateController2();


};
