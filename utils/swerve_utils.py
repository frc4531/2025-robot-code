import math


def step_towards(current: float, target: float, step_size: float) -> float:
    """Steps a value towards a target with a specified step size.

    :param current:  The current or starting value.  Can be positive or negative.
    :param target:   The target value the algorithm will step towards.  Can be positive or negative.
    :param step_size: The maximum step size that can be taken.

    :returns: The new value for {@code current} after performing the specified step towards the specified target.
    """

    if abs(current - target) <= step_size:
        return target

    elif target < current:
        return current - step_size

    else:
        return current + step_size


def step_towards_circular(current: float, target: float, step_size: float) -> float:
    """Steps a value (angle) towards a target (angle) taking the shortest path with a specified step size.

    :param current:  The current or starting angle (in radians).  Can lie outside the 0 to 2*PI range.
    :param target:   The target angle (in radians) the algorithm will step towards.  Can lie outside the 0 to 2*PI range.
    :param step_size: The maximum step size that can be taken (in radians).

    :returns: The new angle (in radians) for {@code current} after performing the specified step towards the specified target.
              This value will always lie in the range 0 to 2*PI (exclusive).
    """

    current = wrap_angle(current)
    target = wrap_angle(target)

    step_direction = math.copysign(target - current, 1)
    difference = abs(current - target)

    if difference <= step_size:
        return target
    elif difference > math.pi:  # does the system need to wrap over eventually?
        # handle the special case where you can reach the target in one step while also wrapping
        if (
            current + math.tau - target < step_size
            or target + math.tau - current < step_size
        ):
            return target
        else:
            # this will handle wrapping gracefully
            return wrap_angle(current - step_direction * step_size)
    else:
        return current + step_direction * step_size


def angle_difference(angle_a: float, angle_b: float) -> float:
    """Finds the (unsigned) minimum difference between two angles including calculating across 0.

    :param angle_a: An angle (in radians).
    :param angle_b: An angle (in radians).

    :returns: The (unsigned) minimum difference between the two angles (in radians).
    """
    difference = abs(angle_a - angle_b)
    return math.tau - difference if difference > math.pi else difference


def wrap_angle(angle: float) -> float:
    """Wraps an angle until it lies within the range from 0 to 2*PI (exclusive).

    :param angle: The angle (in radians) to wrap.  Can be positive or negative and can lie multiple wraps outside the output range.

    :returns: An angle (in radians) from 0 and 2*PI (exclusive).
    """

    two_pi = math.tau

    # Handle this case separately to avoid floating point errors with the floor after the division in the case below
    if angle == two_pi:
        return 0.0
    elif angle > two_pi:
        return angle - two_pi * math.floor(angle / two_pi)
    elif angle < 0.0:
        return angle + two_pi * (math.floor((-angle) / two_pi) + 1)
    else:
        return angle
