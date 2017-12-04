# -*- coding: utf-8 -*-
#!/usr/bin/env python

import numpy as np
import sys
reload(sys)
sys.setdefaultencoding('utf8')
import matplotlib.pyplot as plt
from decimal import Decimal
from math import *

class XP :
    def __init__(self):
        self.WalkedDistance_list = []
        self.Fall_list = []
        self.MaxtrackingError_list = []
        self.DurationOfTheExperiment_list = []
        self.EnergyOfMotors_list = []
        self.EnergyOfWalking_list = []
        self.CostOfTransport_list = []
        self.MechaCostOfTransport_list = []
        self.Froude_list = []
        self.algo = ""
        self.setup = ""
        self.algo_dico = {"10cm":1,"15cm":2,"hwalk":3,"PG":4,"Beam":5,"kawada":6}
        self.setup_dico = {'degrees':1,'Bearing':2,'Pushes':3,'Slopes':4,'Translations':5}
        self.kpi_list = ["WalkedDistance","SuccessRate","MaxtrackingError",
                         "DurationOfTheExperiment","EnergyOfMotors","EnergyOfWalking",
                         "CostOfTransport","MechaCostOfTransport","FroudeNumber"]
        self.dimension_list = ["m","Dimensionless","rad","s","J.m-1.s-1","J.m-1.s-1","Dimensionless",
                               "Dimensionless","Dimensionless"]
        self.success_rate = 0.0

    def __str__(self):
        attrs = vars(self)
        return ', '.join("%s: %s" % item for item in attrs.items())

def isfloat(value):
  try:
    float(value)
    return True
  except ValueError:
    return False

def convert_fall(my_string):
    if my_string=="true":
        return True
    elif my_string=="false":
        return False

def read_file(file_name):
    print "reading file"
    results_file = open(file_name,"r")

    list_lines_str = []
    list_lines_str_split = []
    list_lines_split = []
    header_line = []
    header_file = results_file.readline()

    for line in results_file :
        list_lines_str.append(line)
        #print "list_lines_str",list_lines_str
        list_lines_str_split.append(list_lines_str[-1].split())
        #print "list_lines_str_split",list_lines_str_split
        header_line.append(list_lines_str_split[-1].pop(0))
        #print "header_line",header_line, " list_lines_str_split ", list_lines_str_split
        list_lines_split.append([float(word) for word in list_lines_str_split[-1] if isfloat(word)])
        #for i in len(list_lines_str_split):
        list_lines_split[-1].insert(1,convert_fall(list_lines_str_split[-1][1]))
        #print "list_lines_str_split", list_lines_str_split
        #print "list_lines_split",list_lines_split
    return header_file,header_line,list_lines_split

def discrimin_xp(header_file,header_line,list_lines_split):
    print "discrimin xp"
    xp_list = []
    previous_algo = ""
    previous_setup = 0

    for i in range(len(list_lines_split)) :

        if header_line[i].find("10cm") != -1:
            current_algo = "10cm"
        elif header_line[i].find("15cm") != -1:
            current_algo = "15cm"
        elif header_line[i].find("hwalk") != -1:
            current_algo = "hwalk"
        elif header_line[i].find("PG") != -1:
            current_algo = "PG"
        elif header_line[i].find("Beam") != -1:
            current_algo = "Beam"
        elif header_line[i].find("kawada") != -1:
            current_algo = "kawada"
        else :
            print "no algo pattern found in this line, \n",header_line[i]
            sys.exit(1)
        if previous_algo != current_algo :
            print "new xp detected, algo : ", current_algo
            xp = XP()
            xp.algo = current_algo
            xp_list.append(xp)

        xp_list[-1].WalkedDistance_list.append(list_lines_split[i][0])
        xp_list[-1].Fall_list.append(list_lines_split[i][1])
        xp_list[-1].MaxtrackingError_list.append(list_lines_split[i][2])
        xp_list[-1].DurationOfTheExperiment_list.append(list_lines_split[i][3])
        xp_list[-1].EnergyOfMotors_list.append(list_lines_split[i][4])
        xp_list[-1].EnergyOfWalking_list.append(list_lines_split[i][5])
        xp_list[-1].CostOfTransport_list.append(list_lines_split[i][6])
        xp_list[-1].MechaCostOfTransport_list.append(list_lines_split[i][7])
        xp_list[-1].Froude_list.append(list_lines_split[i][8])

        if header_line[i].find("degrees") != -1:
            deg_index = header_line[i].find("degrees")
            current_setup = header_line[i][(deg_index-2):deg_index]+"°C"
        elif header_line[i].find("Bearing") != -1:
            current_setup = "Bearing"
        elif header_line[i].find("Pushes") != -1:
            current_setup = "Pushes"
        elif header_line[i].find("Slopes") != -1:
            current_setup = "Slopes"
        elif header_line[i].find("Translations") != -1:
            current_setup = "Translations"
        else :
            current_setup = "nothing"
        xp_list[-1].setup = current_setup

        previous_algo = current_algo

        '''xp_list.append([np.mean(WalkedDistance),
                        len(WalkedDistance)/len(Fall), # success rate
                        np.mean(MaxtrackingError),
                        np.mean(DurationOfTheExperiment),
                        np.mean(EnergyOfMotors),
                        np.mean(EnergyOfWalking),
                        np.mean(CostOfTransport),
                        np.mean(MechaCostOfTransport),
                        np.mean(Froude)])'''
        print "xp_list[",i,"] : ", len(xp_list)
    #    print i, xp_list[i]
    return xp_list

def mean_xp(xp_list) :
    list_mean_xp = []
    xp_index_to_rm =[]
    for xp in xp_list :
        print "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
        print xp.algo," ", xp.setup
        print " LEN : ",len(xp.WalkedDistance_list),len(xp.Fall_list),len(xp.MaxtrackingError_list),len(xp.DurationOfTheExperiment_list),len(xp.EnergyOfWalking_list),len(xp.EnergyOfMotors_list),len(xp.CostOfTransport_list),len(xp.MechaCostOfTransport_list),len(xp.Froude_list)
        print "before rm_absurd_values"
        skip_this_xp = rm_absurd_values(xp)
        print " LEN : ",len(xp.WalkedDistance_list),len(xp.Fall_list),len(xp.MaxtrackingError_list),len(xp.DurationOfTheExperiment_list),len(xp.EnergyOfWalking_list),len(xp.EnergyOfMotors_list),len(xp.CostOfTransport_list),len(xp.MechaCostOfTransport_list),len(xp.Froude_list)
        print "after rm_absurd_values"

        #print "nb_of_xp : ", nb_of_xp
        if not skip_this_xp :
            nb_of_xp = len(xp.WalkedDistance_list)
            #print "nb_of_xp : ", nb_of_xp
            list_mean_xp.append((np.mean(xp.WalkedDistance_list),
                            xp.success_rate,
                            np.mean(xp.MaxtrackingError_list),
                            np.mean(xp.DurationOfTheExperiment_list),
                            np.mean(xp.EnergyOfMotors_list),
                            np.mean(xp.EnergyOfWalking_list),
                            np.mean(xp.CostOfTransport_list),
                            np.mean(xp.MechaCostOfTransport_list),
                            np.mean(xp.Froude_list),
                            xp.algo,
                            xp.setup,
                            nb_of_xp))
            print "success rate for ",xp.algo," ", xp.setup," : ",xp.success_rate
            print "xp.WalkedDistance_list :", xp.WalkedDistance_list
        else :
            print "!!!!! no usable value in this xp : ", xp.algo, " ", xp.setup
            xp_index_to_rm.append(xp_list.index(xp))
        #print "nb_of_xp : ", nb_of_xp
    #remove xp with no valid trials:
    for index in reversed(xp_index_to_rm) :
        print "removed xp : ",xp_list[index].algo," ",xp_list[index].setup
        xp_list.pop(index)
    for idx,xp in enumerate(xp_list):
        print xp.algo," ",xp.setup
        #if xp.algo=="15cm" and xp.setup=="10°C":
        #    print "success rate for 15cm 10deg : ", list_mean_xp[idx]

    #print "list_mean_xp : ",list_mean_xp
    return list_mean_xp

def rm_absurd_values(xp):

    absurd_index_list = []
    # remove trials with null walked distance
    if xp.algo=="kawada":
        pass
    else :
        for distance in (xp.WalkedDistance_list):
            if distance == 0 :
                absurd_index_list.append(xp.WalkedDistance_list.index(distance))
                print "+ experiment will be removed in ", xp.algo, " ", xp.setup
                print "+ walked distance is 0 , index : ",absurd_index_list[-1]
        for absurd_index in reversed(absurd_index_list):
            xp.WalkedDistance_list.pop(absurd_index)
            xp.Fall_list.pop(absurd_index)
            xp.MaxtrackingError_list.pop(absurd_index)
            xp.DurationOfTheExperiment_list.pop(absurd_index)
            xp.EnergyOfMotors_list.pop(absurd_index)
            xp.EnergyOfWalking_list.pop(absurd_index)
            xp.CostOfTransport_list.pop(absurd_index)
            xp.MechaCostOfTransport_list.pop(absurd_index)
            xp.Froude_list.pop(absurd_index)
    if len(xp.WalkedDistance_list) == 0:
        return True  # skip_this_xp

    # remove trials duration over 200s
    absurd_index_list = []
    #print "before duration over 200 : ", xp.WalkedDistance_list
    if xp.algo=="kawada" or xp.setup=="Slopes":
        pass
    else :
        for duration in (xp.DurationOfTheExperiment_list):
            if duration > 200:
                absurd_index_list.append(xp.DurationOfTheExperiment_list.index(duration))
                print "absurd index : ", absurd_index_list[-1]
                print "# experiment has been removed in ", xp.algo, " ", xp.setup
                print "# duration over 200 (Translations and slopes excluded) : ", duration
        for absurd_index in reversed(absurd_index_list):
            xp.WalkedDistance_list.pop(absurd_index)
            xp.Fall_list.pop(absurd_index)
            xp.MaxtrackingError_list.pop(absurd_index)
            xp.DurationOfTheExperiment_list.pop(absurd_index)
            xp.EnergyOfMotors_list.pop(absurd_index)
            xp.EnergyOfWalking_list.pop(absurd_index)
            xp.CostOfTransport_list.pop(absurd_index)
            xp.MechaCostOfTransport_list.pop(absurd_index)
            xp.Froude_list.pop(absurd_index)
    if len(xp.WalkedDistance_list)==0:
        return True #skip_this_xp

    #remove trials with duration far away of the others
    absurd_index_list = []
    duration_variance = np.var(xp.DurationOfTheExperiment_list)
    duration_mean = np.mean(xp.DurationOfTheExperiment_list)
    #print "xp.DurationOfTheExperiment_list : ", xp.DurationOfTheExperiment_list
    for duration in (xp.DurationOfTheExperiment_list) :
        if abs(duration-duration_mean) > 3*sqrt(duration_variance):
            absurd_index_list.append(xp.DurationOfTheExperiment_list.index(duration))
            print "absurd index : ", absurd_index_list[-1]
            print "* experiment has been removed in ",xp.algo," ",xp.setup
            print "* duration over 3 sigma : ",abs(duration-duration_mean)," > 3 * ",sqrt(duration_variance)
    for absurd_index in reversed(absurd_index_list):
        xp.WalkedDistance_list.pop(absurd_index)
        xp.Fall_list.pop(absurd_index)
        xp.MaxtrackingError_list.pop(absurd_index)
        xp.DurationOfTheExperiment_list.pop(absurd_index)
        xp.EnergyOfMotors_list.pop(absurd_index)
        xp.EnergyOfWalking_list.pop(absurd_index)
        xp.CostOfTransport_list.pop(absurd_index)
        xp.MechaCostOfTransport_list.pop(absurd_index)
        xp.Froude_list.pop(absurd_index)
    if len(xp.WalkedDistance_list)==0:
        return True #skip_this_xp

    # calculate success rate and remove xp without any success
    temp_Fall_list = {i: xp.Fall_list.count(i) for i in xp.Fall_list}
    print temp_Fall_list
    try:
        xp.success_rate = temp_Fall_list[False] / float(len(xp.Fall_list))  # hasn't  fallen
    except:
        try:
            xp.success_rate = (len(xp.WalkedDistance_list) - temp_Fall_list[True]) / float(len(xp.Fall_list))
        except:
            print "0 success in this xp : ", xp.algo, " ", xp.setup
            return True

    # remove trials where the robot has fallen
    if xp.algo=="kawada" or xp.setup=="Pushes"or (xp.algo=="PG" and xp.setup=="10°C"):
        pass
    else :
        for idx,fall in enumerate(xp.Fall_list):
            if fall == True:
                absurd_index_list.append(idx)
                print "absurd index : ", absurd_index_list[-1]
                print "* experiment has been removed in ", xp.algo, " ", xp.setup
                print "* robot fell in this xp (walked distance) : ", xp.WalkedDistance_list[idx]
        print "nb of trials to remove, len xp : ", len(absurd_index_list), " ", len(xp.Fall_list)
        for absurd_index in reversed(absurd_index_list):
            xp.WalkedDistance_list.pop(absurd_index)
            xp.Fall_list.pop(absurd_index)
            xp.MaxtrackingError_list.pop(absurd_index)
            xp.DurationOfTheExperiment_list.pop(absurd_index)
            xp.EnergyOfMotors_list.pop(absurd_index)
            xp.EnergyOfWalking_list.pop(absurd_index)
            xp.CostOfTransport_list.pop(absurd_index)
            xp.MechaCostOfTransport_list.pop(absurd_index)
            xp.Froude_list.pop(absurd_index)
        # print "xp.DurationOfTheExperiment_list : ", xp.DurationOfTheExperiment_list
    if len(xp.WalkedDistance_list) == 0:
        return True  # skip_this_xp
    else :
        return False #can continue this xp


def plot_graph(list_mean_xp,xp_list) :
    xp_tmp=XP()
    for key in xp_tmp.algo_dico.keys() : #loop on algo
        fig, ax = plt.subplots(3, 3)
        plt.suptitle("Algorithm : "+key)
        jk=0 #place of suplot
        for j in range(3): #1st loop place subplot
            for k in range(3): #2nd loop subplot
                y_list=[xp[jk] for xp in list_mean_xp if xp_list[list_mean_xp.index(xp)].algo==key] # get mean values for algo
                setup_list=[xp[-2] for xp in list_mean_xp if xp_list[list_mean_xp.index(xp)].algo==key]#get setup found for algo
                nb_of_xp_list=[xp[-1] for xp in list_mean_xp if xp_list[list_mean_xp.index(xp)].algo == key]  # get setup found for algo
                print "y_list",y_list
                #plt.plot(y_list)
                #plt.show()
                y_tuple = tuple(y_list)
                setup_tuple = tuple(setup_list)
                N = len(y_tuple)

                ind = np.arange(N)  # the x locations for the groups
                width = 0.35  # the width of the bars

                #fig, ax = plt.subplots()
                #ax[0, 0].plot(x, y)
                rects1 = ax[j, k].bar(ind, y_tuple, width, color='r')

                # add some text for labels, title and axes ticks
                ax[j, k].set_ylabel(xp_tmp.dimension_list[jk])
                ax[j, k].set_title(xp_tmp.kpi_list[jk])
                ax[j, k].set_xticks(ind)
                print "setup_tuple : ",setup_tuple
                ax[j, k].set_xticklabels(setup_tuple)
                ax[j, k].set_ylim((ax[j, k].get_ylim()[0],ax[j, k].get_ylim()[1]*1.1))
                nb_points = len(y_list)

                autolabel(rects1,ax,j,k,nb_of_xp_list)

                plt.show(block=False)
                jk+=1
                print "end loop one plot "

    return

def autolabel(rects,ax,j,k,nb_of_xp_list):
    """
    Attach a text label above each bar displaying its height
    """
    for rect in rects:
        height = rect.get_height()
        if height> 0.1:
            ax[j, k].text(rect.get_x() + rect.get_width()/2., height,
                    '%s \nnb:%s' % (str('{0:.2f}'.format(height)),str(nb_of_xp_list[rects.index(rect)])),
                    ha='center', va='bottom')
        else:
            ax[j, k].text(rect.get_x() + rect.get_width() / 2., height,
                      '%s \nnb:%s' % (str('{0:.2E}'.format(height)),str(nb_of_xp_list[rects.index(rect)])),
                      ha='center', va='bottom')

def close_figures():
    for i in plt.get_fignums():
        plt.close(i)
    print "all figures closed"

if __name__ == '__main__':
    close_figures()
    if len(sys.argv)>1:
        file_name = sys.argv[1]#"/home/anthropobot/devel/EnergyComputation/_build/bin/DEBUG/results_2017_Oct_19.txt"
    else :
        exit(1)
    header_file, header_line, list_lines_split = read_file(file_name)
    xp_list = discrimin_xp(header_file, header_line, list_lines_split)
    list_mean_xp = mean_xp(xp_list)
    plot_graph(list_mean_xp, xp_list)



