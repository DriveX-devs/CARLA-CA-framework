# -*- coding: utf-8 -*-
"""
The CA demo with local collision avoidance only (no simulated network): the
application of run_ca_extended.py in mode 1, kept under its historical name.

    python run_ca.py                          # SiMO on LiDAR
    python run_ca.py --seconds 20 --simo-cooperation ideal
    python run_ca.py --perception yolo --model n

Every option of run_ca_extended.py applies, except --app-mode.
"""
import os
import sys

if __name__ == '__main__':
    if any(a == '--app-mode' or a.startswith('--app-mode=')
           for a in sys.argv[1:]):
        sys.exit('run_ca.py is the local-CA demo (app mode 1): use '
                 'run_ca_extended.py to choose a mode')
    script = os.path.join(os.path.dirname(os.path.realpath(__file__)),
                          'run_ca_extended.py')
    os.execv(sys.executable, [sys.executable, script, '--app-mode', '1'] +
             sys.argv[1:])
