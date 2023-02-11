class MathUtils {
 public:
  /**
   * Rounds (floors) a value to the specified step size.
   * For instance, this can be used to discretize continuous inputs
   * to a specific values.
   *
   * @param value The value to round.
   * @param step The step size.
   * @return The rounded value.
   */
  static double RoundToStep(double value, double step);
};