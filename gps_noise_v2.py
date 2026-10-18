STOPPED_BELOW = 0.5
MOVING_ABOVE = 2.0
    

F9P_IDENTICAL_ZONES = {
    "stopped" : {
        "sigma" : None,
        "tau" : None,
        "ratio" : None
    },
    "moving" : {
        "sigma" : None,
        "tau" : None,
        "ratio" : None
    }
}

M8T_IDENTICAL_MOVING = {
    "sigma" : None,
    "tau" : None,
    "ratio" : None,
    "step_rate" : None,
    "step_size" : None
}


RECEIVERS = {
    "f9p" : {
        "MILD" : F9P_IDENTICAL_ZONES,
        "MODERATE" : F9P_IDENTICAL_ZONES,
        "STRONG" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None
                },
                "moving" : {
                    "sigma" : None,
                    "tau" : None,
                    "ratio" : None
                    },
        }
    },
    "m8t" : {
        "MILD" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None
            },
            "moving" : M8T_IDENTICAL_MOVING,
        },
        "MODERATE" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None
            },
            "moving" : M8T_IDENTICAL_MOVING,
        },
        "STRONG" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None
            },
            "moving" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None
            },
        }
    }
}
ZONES = [
    {
        "name" : "bridge",
        "x_min" : 0,
        "x_max" : 30,
        "y_min" : 128,
        "y_max" : 145,
        "level" : "NO_SIGNAL"
    },
    {
        "name" : "north road",
        "x_min" : -120,
        "x_max" : 115,
        "y_min" : -80,
        "y_max" : -45,
        "level" : "STRONG"
    },

    {
        "name" : "NW corner + west road",
        "x_min" : -120,
        "x_max" : -85,
        "y_min" : -80,
        "y_max" : 25,
        "level" : "STRONG"
    },

    {
        "name" : "inner road y~65",
        "x_min" : -50,
        "x_max" : 90,
        "y_min" : 58,
        "y_max" : 75,
        "level" : "STRONG"
    },

    {
        "name" : "SW corner + south road",
        "x_min" : -120,
        "x_max" : -40,
        "y_min" : 85,
        "y_max" : 145,
        "level" : "MILD"
    },

    {
        "name" : "rest of town",
        "x_min" : -130,
        "x_max" : 125,
        "y_min" : -85,
        "y_max" : 155,
        "level" : "MODERATE"
    }
]

def zone_level(x, y):
    for zone in ZONES:
        if ((zone["x_min"] <= x <=zone["x_max"]) and (zone["y_min"] <= y <= zone["y_max"])):
            return zone["level"]
    raise ValueError(f"Error values {x}, {y}")


def motion_state(speed, previous):
    if speed < STOPPED_BELOW:
        current_state = "stopped"
        return current_state
    elif speed > MOVING_ABOVE:
        current_state = "moving"
        return current_state
    else: return previous