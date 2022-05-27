# -*- coding: utf-8 -*-
"""
@file parser_temps_reel.py

@brief TOP AERO - zephyr.sol : parsage temps réel des données de la télémétrie

@author Mohamed-Iadh BANI <mohamed-iadh.bani@top-aero.com>

@todo confronter le script à la cadence de l'émission
"""

import serial
import io

################################################################################

class Parser:

    def __init__(self, port, baud, timeout, file_out):
        self.__s = serial.Serial(port, baud, timeout=timeout)
        self._fo = open(file_out, 'bw')

        # lecteur d'eol robuste
        self.__sio = io.TextIOWrapper(io.BufferedRWPair(self.__s, self.__s))

    def readline(self):
        self.__s.flush()
        ligne = self.__s.readline()

        if not ligne:
            return []

        alt, acx, acy, acz, tps, gps, *psi = ligne

        # schéma de parsage normal : temps, altitude, accélération
        alt, acx, acy, acz, tps = alt[4:], acx[4:], acy[4:], acz[4:], tps[4:]

        # schéma de parsage pressions
        psi = [e[4:] for e in psi]

        # écriture fichier
        self._fo.write(ligne)

        return alt, acx, acy, acz, tps, gps, psi
