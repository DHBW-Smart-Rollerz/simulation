class Config:
    road_width = 0.4

    TURN_SF_MARK_WIDTH = 0.072
    TURN_SF_MARK_LENGTH = 0.5
    ROAD_MARKING_DISTANCE = 0.2
    # 0.3 -0.5 according to regulations
    PRIORITY_SIGN_DISTANCE_INTERVALL = (0.3, 0.5)
    # 0.15-0.25 according to regulations
    TURN_SIGN_DISTANCE_INTERVALL = (0.15, 0.25)
    # 0.05-0.25 according to regulations
    SURFACE_MARKING_DISTANCE_INTERVALL = (0.05, 0.25)
    # 0.075 - 0.125 according to regulations
    SIGN_ROAD_PADDING_INTERVALL = (0.075, 0.125)

    @staticmethod
    def _interpolate(interval, rand):
        """Map rand in [0, 1] linearly onto the interval.

        Args:
            interval: (lower, upper) bound tuple.
            rand: Position inside the interval; 0 gives lower, 1 gives upper.

        Returns:
            float -- The interpolated value.
        """
        lower, upper = interval
        return lower + (upper - lower) * rand

    @staticmethod
    def get_prio_sign_dist(rand=0.5):
        return Config._interpolate(Config.PRIORITY_SIGN_DISTANCE_INTERVALL, rand)

    @staticmethod
    def get_turn_sign_dist(rand=0.5):
        return Config._interpolate(Config.TURN_SIGN_DISTANCE_INTERVALL, rand)

    @staticmethod
    def get_surface_mark_dist(rand=0.5):
        return Config._interpolate(Config.SURFACE_MARKING_DISTANCE_INTERVALL, rand)

    @staticmethod
    def get_sign_road_padding(rand=0.5):
        return Config._interpolate(Config.SIGN_ROAD_PADDING_INTERVALL, rand)
