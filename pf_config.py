"""
Ρυθμίσεις του particle_filter.py: διαβάζει το config.yaml και τα flags της γραμμής εντολών (15 Νοε 2026).

Κανόνας: ό,τι γράφει το config.yaml ισχύει, εκτός αν δώσεις flag· το flag κερδίζει.
Κάθε ρύθμιση του αρχείου γίνεται μόνη της flag με το ίδιο όνομα (lane_sigma → --lane-sigma).
Οι διακόπτες (true / false) ανοίγουν με --lanes και κλείνουν με --no-lanes.

    from pf_config import load_settings
    cfg = load_settings()      # cfg.receiver, cfg.lane_sigma, cfg.camera_noise, ...
"""

import argparse
import os
from types import SimpleNamespace

import yaml

CONFIG_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "config.yaml")
RECEIVERS = ["f9p", "m8t"]
CHOICES = {"receiver": RECEIVERS, "lane_source": ["fake", "detector"], "gps_level": ["MILD", "MODERATE", "STRONG"]}
WHOLE_NUMBERS = {"seed", "particles", "lane_every"}
HELP = {"csv": "ποια καταγραφή (όνομα αρχείου στον φάκελο drives/)", "seed": "τα «ζάρια» του φίλτρου και του θορύβου"}


def read_file():
    """Το config.yaml όπως είναι, χωρίς flags."""
    with open(CONFIG_FILE, encoding="utf-8") as f:
        return yaml.safe_load(f)


def flat(config):
    """Όλες οι ρυθμίσεις σε ένα επίπεδο: {όνομα: τιμή}. Οι ρυθμίσεις ανά δέκτη (filter.f9p, filter.m8t) μένουν έξω."""
    settings = {}
    for key, value in config.items():
        items = value.items() if isinstance(value, dict) else [(key, value)]
        for name, v in items:
            if name in RECEIVERS:
                continue
            if name in settings:
                raise ValueError(f"config.yaml: η ρύθμιση «{name}» υπάρχει δύο φορές")
            settings[name] = v
    return settings


def load_settings(argv=None):
    """Οι τελικές ρυθμίσεις: config.yaml και από πάνω τα flags. Επιστρέφει αντικείμενο με μία ιδιότητα ανά ρύθμιση."""
    config = read_file()
    settings = flat(config)
    per_receiver = sorted(config["filter"][RECEIVERS[0]])        # pos_sigma, gps_sigma, heading_gate

    parser = argparse.ArgumentParser(description="Particle filter. Οι προεπιλογές είναι στο config.yaml.")
    for name, value in settings.items():
        flag = "--" + name.replace("_", "-")
        if isinstance(value, bool):
            parser.add_argument(flag, action=argparse.BooleanOptionalAction, default=None)
        else:
            # αριθμοί: δεκαδικοί, εκτός από όσους είναι πλήθος· οι κενές ρυθμίσεις (null) είναι αριθμοί
            kind = str if isinstance(value, str) else int if name in WHOLE_NUMBERS else float
            parser.add_argument(flag, type=kind, default=None, choices=CHOICES.get(name), help=HELP.get(name))
    for name in per_receiver:
        parser.add_argument("--" + name.replace("_", "-"), type=float, default=None,
                            help="αντικαθιστά την τιμή του δέκτη που τρέχει")
    args = vars(parser.parse_args(argv))

    for name in settings:
        if args[name] is not None:
            settings[name] = args[name]
    for name in per_receiver:                                    # οι τιμές του δέκτη που διαλέχτηκε
        value = config["filter"][settings["receiver"]][name]
        settings[name] = value if args[name] is None else args[name]
    return SimpleNamespace(**settings)


def print_settings(cfg):
    """Τυπώνει τις ρυθμίσεις με τις οποίες έτρεξε το φίλτρο, σε μία γραμμή η καθεμία."""
    print("ρυθμίσεις:")
    for name, value in vars(cfg).items():
        print(f"  {name}: {value}")
