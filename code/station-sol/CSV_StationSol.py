#importation des bibliothèques
import csv

#Ouverture du fichier contenant les données
file = open("test.txt",'rt')
rawdata = file.read()

"""
entries : list[str]
liste contenant chaque ligne de rawdata
"""
entries = rawdata.split("\n")
#print(entries)

#fermetrue du fichier initiale
file.close()

"""
final_entries : list[str]
On supprime les éléments d'entries qui ne contiennent 
pas les éléments à garder
"""
final_entries = [elem for elem in entries if len(elem)>7]
#print(entries)

"""
tab : list[list[str]]
On sépare deux fois entries avec les virgules
Les deux premiers éléments de entrie[i] sont à séparer
"""
tab = [elem.split(',',2) for elem in final_entries]
#print(tab)

"""
data : list[str]
Contient à chaque indice les données voulu
"""
data = [line[2] for line in tab]
#print(data)

"""
tab_data : list[list[str]]
A partir de la liste data on créé un tableau
où chaque données à récupérer de chaque lignes
on été séparé
"""
tab_data = [elem.split(' ',3) for elem in data]
#print(data)


"""
alt : list[float]
acl : list[float]
t : list[float]
gps : list[str]
Chaque listes contiennent les colones correspondant 
aux données récupérer
"""
alt = [float(elem[0][4:]) for elem in tab_data]
acl = [float(elem[1][4:]) for elem in tab_data]
t = [float(elem[2][2:]) for elem in tab_data]
gps = [elem[3][4:] for elem in tab_data]

"""
Colname : list:[str]
le nom de chaques colonnes
"""
Colname = ["Temps", "Altitude", "Accélération", "GPS"]

"""
Col : list[list[float]]
liste des colonnes
"""
Col = [t,alt,acl,gps]

#Début de l'écriture dans le fichier .csv
file = open("data.csv", "w", newline='')
writerCSV = csv.writer(file, delimiter=";")

#écriture de la première ligne (nom des colonnes)
writerCSV.writerow(Colname)

#écritures de chaques lignes
for num in range(len(alt)):
    line = []
    for ind in range(len(Col)):
        line.append(Col[ind][num])
        
    writerCSV.writerow(line)
    
#fermeture du fichier
file.close()       
    

"""
Note de Adrien du 28/04/2021

Points à corriger (pas ultimement nécessaire) :
- le programme n'est pas très efficace (trop de boucles/compréhensions)
"""