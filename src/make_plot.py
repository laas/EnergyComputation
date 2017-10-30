#!/usr/bin/env python
import numpy as np
import sys
import matplotlib.pyplot as plt

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
                         "CostOfTransport","MechaCostOfTransport","Froude"]

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
            current_setup = header_line[i][(deg_index-2):deg_index]+"degrees"
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
    #for i in range(len(xp_list)) f, axarr = plt.subplots(2, 2):
    #    print i, xp_list[i]
    return xp_list

def mean_xp(xp_list) :
    list_mean_xp = []
    for xp in xp_list :
        list_mean_xp.append((np.mean(xp.WalkedDistance_list),
                        len(xp.WalkedDistance_list)/len(xp.Fall_list), # success rate
                        np.mean(xp.MaxtrackingError_list),
                        np.mean(xp.DurationOfTheExperiment_list),
                        np.mean(xp.EnergyOfMotors_list),
                        np.mean(xp.EnergyOfWalking_list),
                        np.mean(xp.CostOfTransport_list),
                        np.mean(xp.MechaCostOfTransport_list),
                        np.mean(xp.Froude_list),
                        xp.algo,
                        xp.setup))
        print "np.mean(xp.CostOfTransport_list) : ",np.mean(xp.CostOfTransport_list)
        if np.mean(xp.CostOfTransport_list) < 0 :
            print "list : " , xp.CostOfTransport_list
            print xp
    print "list_mean_xp : ",list_mean_xp
    return list_mean_xp

def plot_graph(list_mean_xp,xp_list) :
    xp_tmp=XP()
    i=0
    for key in ["hwalk"] :#xp_tmp.algo_dico.keys() :
        plt.figure(i)
        f, ax = plt.subplots(3, 3)
        for j in range(3):
            for k in range(3):
                jk=j+k
                y_list=[xp[jk] for xp in list_mean_xp if xp_list[list_mean_xp.index(xp)].algo==key] #cot in hwalk
                setup_list=[xp[-1] for xp in list_mean_xp if xp_list[list_mean_xp.index(xp)].algo==key]#setpu in hwalk
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
                ax[j, k].set_ylabel('Scores')
                ax[j, k].set_title(xp_tmp.kpi_list[jk]+' for '+key)
                ax[j, k].set_xticks(ind) #ax.set_xticks(ind + width / 2)
                print "setup_tuple : ",setup_tuple
                ax[j, k].set_xticklabels(setup_tuple)

                #ax.legend((rects1[0], rects2[0]), ('Men', 'Women'))

                autolabel(rects1,ax,j,k)
                #autolabel(rects2)

                plt.show(block=False)
                i+=1
                print "end loop one plot "


    return

def autolabel(rects,ax,j,k):
    """
    Attach a text label above each bar displaying its height
    """
    for rect in rects:
        height = rect.get_height()
        ax[j, k].text(rect.get_x() + rect.get_width()/2., height,
                '%s' % str('{0:.3f}'.format(height)),
                ha='center', va='bottom')

def close_figures():
    for i in plt.get_fignums():
        plt.close(i)
    print "all figures closed"

if __name__ == '__main__':
    close_figures()
    file_name = "/home/anthropobot/devel/EnergyComputation/_build/bin/DEBUG/results_2017_Oct_19.txt"
    header_file, header_line, list_lines_split = read_file(file_name)
    xp_list = discrimin_xp(header_file, header_line, list_lines_split)
    list_mean_xp = mean_xp(xp_list)
    plot_graph(list_mean_xp, xp_list)




