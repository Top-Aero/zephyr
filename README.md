# Zéphyr

![forthebadge](https://svgshare.com/i/iZj.svg) ![forthebadge](https://forthebadge.com/images/badges/powered-by-coffee.svg) ![badge](https://forthebadge.com/images/badges/contains-tasty-spaghetti-code.svg)

---

## Description

Le vent d'Ouest, fils d'Astreos et d'Éos, attise le feu de notre inspiration : Zéphyr. Projet phare de l'association [Top Aéro](https://www.top-aero.com/), il s'agit d'une fusée expérimentale transsonique dont la coiffe est équipée de capteurs de pression, afin d'observer l'écoulement de l'air qui l'entoure, puis d'en comparer les résultats à une simulation numérique faite préalablement.

Le dépôt Git contient les conceptions des PCB du système électronique et les codes des microcontrôleurs contenus dans la case électronique de la fusée. Ils sont respectivement contenus dans `case_elec` et `code`.

### Structure du dossier `case_elec`

```
case_elec
├───odb
│   └───odb-backups
├───ressources
└───seq
    └───seq-backups
```

Les sous-dossiers odb et seq, désignant respectivement l'ordinateur de bord et le séquenceur embarqués dans la fusée, contiennent des projets KiCAD, le logiciel de CAO pour l'électronique qui a été utilisé durant la phase de conception. Les fichiers de conception, d'extension kicad_pro, kicad_sch et kicad_pcb suffisent à décrire ce qui a été conçu, mais le fichier .gitignore que nous avions rédigé (il sert à filtrer les documents inutiles dans le workflow Git) a été insuffisant. Avec le temps et le ménage qui n'est pas fait, des documents supplémentaires finissent par encombrer les dossiers KiCAD. Il est tout de même intéressant de les décrire :

- les `*.gbr` sont les fichiers de fabrication, générés par le logiciel ;
- les `*.step` sont les fichiers de visualisation 3D de la carte électronique et des composants ;
- les dossiers `*-backups` contiennent les versions précédentes au format zip d'un même projet.

Les -backups se sont avérés vitaux à une ou deux reprises lorsque nous supprimions des éléments critiques par inadvertance. Bien qu'inutiles en soi, ils compensaient le manque de maîtrise que l'on avait de Git et de KiCAD et faisaient gagner beaucoup de temps. À l'avenir, il faudra penser à configurer KiCAD pour ne conserver que la dernière version, car c'est la plus utile en cas de pépin.

#### Structure du dossier `ressources`

```
ressources
├───3Dmodels
│   └───teensy
├───lib_fp
│   ├───Adafruit BNO055
│   │   ├───Adafruit BNO055-backups
│   │   └───Adafruit BNO055.pretty
│   ├───Adafruit Ultimate GPS
│   │   └───Adafruit Ultimate GPS-backups
│   ├───Adafruit_sensors.pretty
│   ├───Adafruit_Ultimate_GPS
│   │   ├───Adafruit_Ultimate_GPS-backups
│   │   └───Adafruit_Ultimate_GPS.pretty
│   ├───common_misc.pretty
│   ├───MPL3115A2_v0.2
│   │   ├───MPL3115A2_v0.2-backups
│   │   └───MPL3115A2_v0.2.pretty
│   ├───Reyax.pretty
│   └───teensy.pretty
├───lib_sch
└───tmp_resources
    ├───Adafruit-BNO055-Breakout-PCB-master
    │   └───assets
    ├───Adafruit-MPL3115A2-PCB-master
    │   └───assets
    ├───prj
    │   └───altimetre
    └───zip
```

Le dossier ressources se structure ainsi :

- 3Dmodels : contient les modèles 3D des composants. Utile pour la visualisation et l'export au format .step ;
- lib_fp : *library footprints*, contient les empreintes des composants utilisés, c'est-à-dire leur contour et les trous à placer sur la carte électronique ;
- lib_sch : *library schematics*, contient les symboles électriques utilisés dans les schémas électriques ;
- liensUtiles.txt : contient des liens vers des bibliothèques d'empreintes, de symboles électriques et de modèles 3D.
- tmp_ressources : un dossier un peu fouillis qu'il faudrait prendre le temps de réhabiliter.

Le fichier liensUtiles.txt porte mal son nom, car les sites vers lesquels il renvoie se sont avéré moins utiles qu'espérés : des composants complexes comme les capteurs Adafruit n'y sont pas documentés. Il a fallu, et il a mieux valu concevoir par nous même les empreintes, même approximativement. Par ailleurs, Adafruit met à disposition des modèles 3D de leurs composants sur [cette page GitHub](https://github.com/adafruit/Adafruit_CAD_Parts). Toutes les entreprises n'en font pas autant.

### Structure du dossier `code`

```
code
├───seq
├───station-sol
└───telemetrie
    ├───TopAeroEM
    ├───TopAeroEM0720
    ├───TopAeroEM_test_pitot
    └───TopAeroRV
```

Les dossiers seq, TopAEroEM (EM pour émission) et TopAéroRV (RV pour récéption...) dans telemetrie contiennent des fichiers ino. C'est le format de fichier que reconnaît l'éditeur Arduino que l'on utilisait pour programmer les microcontrôleurs de la case électronique. Pour plusieurs raisons, il faut lui préférer à l'avenir son alternative PlatfomIO soit en tant qu'extension, soit en standalone à intégrer au workflow de votre IDE de choix. Pour les experts, un makefile Arduino peut suffire.

Plusieurs dossiers TopAeroEM se côtoient. Une convention de nommage récurrente est de suffixer à une autre version d'un même dossier le mois puis le jour où cette version a vu le jour, en quatre chiffres. Ainsi, le dossier TopAeroEM0720 est une version de TopAeroEM créée un 20 juillet. Des exceptions subsistent (TopAeroEM_test_pitot), mais tant qu'elles ne sont pas nombreuses (plus que 2), elles restent gérables. Ces deux pratiques sont tout de même à éviter, car elles ne tirent pas profit de la technologie de versionnage de Git. Il faut préférer mettre à jour le code existant, ou encore ajouter une nouvelle branche au projet, ou dans le pire des cas, se passer complètement du workflow Git si le contraire amenait à désorganiser la structure du projet.

Le dossier station-sol contient plusieurs scripts qui n'ont pas pu être déployés. Le seul script utilisable en l'état est simpleUSBlogger.py qui se charge d'écrire dans un fichier les données réceptionnées.

## Derniers mots

La fusée a décollé un jour de juillet 2022. À part quelque modifications mineures et sporadiques sur sa page GitHub, le projet est terminé.

## Contacts

Nom               | Rôle                              | Contact
------------------|-----------------------------------|----------
Mohamed-Iadh Bani | Conception séquenceur             | [:e-mail:](mailto:mohamed-iadh.bani@top-aero.com) [:octocat:](https://github.com/mediadhBani)
Théo Tugayé       | Conception ordinateur de bord     | [:e-mail:](mailto:theo.tugaye@top-aero.com) [:octocat:](https://github.com/Alhucarr)
