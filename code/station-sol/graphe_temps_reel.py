# -*- coding: utf-8 -*-
"""
@file graphe_dq_tps_reel.py

@brief TOP AERO - zephyr.sol : graphe temps reel mesures receptionnees

@author Mohamed-Iadh BANI <mohamed-iadh.bani@top-aero.com>

@todo confronter le script à la cadence de l'emission
@todo utiliser `randrange` sans alias deroutant
"""

from collections import deque
from itertools import count
from matplotlib.animation import FuncAnimation
# from matplotlib.widgets import TextBox
import matplotlib.pyplot as plt
import parser_temps_reel as ptr
from random import randrange as randint

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

# Constantes generales

BAUD = 9600
DEMO = True
FULLSCREEN = False
FICHIER = 'zephyr_reception.bin'
NBR_LIGNES_GPS = 10
NBR_MESURES = 100
NBR_PSI = 2
PORT = '/dev/ttyACM0'

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

# Quadrillage de la toile

fig = plt.figure()

manager = plt.get_current_fig_manager()

if FULLSCREEN:
    manager.full_screen_toggle()
else:
    manager.window.state('zoomed')

# emplacemments des widgets de chaque mesure
ax_acx = fig.add_axes([0, .5, .33, .5])    # acceleration x
ax_acy = fig.add_axes([.33, .5, .34, .5])  # acceleration y
ax_acz = fig.add_axes([.67, .5, .33, .5])  # acceleration z
ax_alt = fig.add_axes([0, .1, 1, .4])      # altitude
ax_gps = fig.add_axes([.5, 0, .5, .1])     # derniere ligne GPS
ax_psi = fig.add_axes([0, 0, .5, .1])      # pressions

# titres
ax_acx.set_title("Acceleration X [m/s$^2$]", y=.9)
ax_acy.set_title("Acceleration Y [m/s$^2$]", y=.9)
ax_acz.set_title("Acceleration Z [m/s$^2$]", y=.9)
ax_alt.set_title("Altitude [m]", y=.9)

# ajouter traits de mesure ticks
for ax in [ax_acx, ax_acy, ax_acz, ax_alt]:
    ax.tick_params(axis='both', direction='in', pad=-23)

ax_psi.set_axis_off()  # retirer les axes autour du tableau de pressions
# ax_gps dans la suite

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

# declaration des containers de mesures

dq_tps = deque([0], maxlen=NBR_MESURES) if DEMO else deque(maxlen=NBR_MESURES)
dq_acx = dq_tps.copy()
dq_acy = dq_tps.copy()
dq_acz = dq_tps.copy()
dq_alt = deque([0], maxlen=NBR_MESURES) if DEMO else deque(maxlen=NBR_MESURES)

index = count()  # utile seulement pour la demo

dq_psi = deque([float('nan') for _ in range(NBR_PSI)], maxlen=NBR_PSI)

# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

# Affichage de contenu

# lignes des graphes
line_acx, = ax_acx.plot(dq_tps, dq_acx)
line_acy, = ax_acy.plot(dq_tps, dq_acy)
line_acz, = ax_acz.plot(dq_tps, dq_acz)
line_alt, = ax_alt.plot(dq_tps, dq_alt)

# tableau des pressions
lbl_psi = ["Pression 11 [hPa]",  "Pression 12 [hPa]"]  # etiquettes pressions
tab_psi = plt.table(cellText=[dq_psi], colLabels=lbl_psi, bbox=[0, 0, 1, 1])

# ligne de gps
tx_gps = ax_gps.text(.05, .4, "GPS ", fontsize=11)

# NOTE ecriture inetressante pour colorer les cellules du tableau dynamiquement
# ```tab_psi = ax_psi.imshow(dq_psi, aspect='auto')```


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

# fonctions de l'animation

def rafraichissement(txt_gps=""):
    """affiche les nouvelles donnees"""

    # mise à jour graphes
    line_acx.set_data(dq_tps, dq_acx)
    line_acy.set_data(dq_tps, dq_acy)
    line_acz.set_data(dq_tps, dq_acz)
    line_alt.set_data(dq_tps, dq_alt)

    # mise à jour pressions
    for i in range(NBR_PSI):
        tab_psi[1, i].set_text_props(text=dq_psi[i])

    # mise à jour GPS
    tx_gps._text = txt_gps

    for ax in [ax_acx, ax_acy, ax_acz, ax_alt]:
        ax.relim()
        ax.autoscale_view(True, True, True)


def demo_animation(i):
    """Demonstration du script"""

    dq_tps.append(next(index))
    dq_acx.append(randint(0, 5))
    dq_acy.append(randint(0, 5))
    dq_acz.append(randint(0, 5))
    dq_alt.append(dq_alt[-1] + randint(-1, 2))
    dq_psi.extend([1013 + randint(-10, 11) for _ in range(NBR_PSI)])

    rafraichissement()


def vraie_animation(i):
    """Trace dq_tps reel à partir d'un ficher dejà mis en forme par le code du
        parser.
    """
    ligne = reception.readline()

    if ligne == []:
        ligne = reception.readline()

    # alt acx acy acz tps gps p1 p2
    alt, acx, acy, acz, tps, gps, psi = ligne
    # alt, acx, acy, acz, tps, gps, psi =

    dq_alt.append(float(alt))
    dq_acx.append(float(acx))
    dq_acy.append(float(acy))
    dq_acz.append(float(acz))
    dq_tps.append(float(tps))
    tab_psi.extend([float(e) for e in psi])

    rafraichissement(gps)


# %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% #

if __name__ == '__main__':
    # necessaire pour la fonction de parallelisme

    if DEMO:
        next(index)
        ani = FuncAnimation(fig, demo_animation, 200)

    else:
        reception = ptr.Parser(PORT, BAUD, timeout=1, file_out=FICHIER,
                               n_pressions=NBR_PSI)
        ani = FuncAnimation(fig, vraie_animation, 200)

    plt.show()
