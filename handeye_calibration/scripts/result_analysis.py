#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
This is used for analyzing the results of hand-eye calibration.
"""

import numpy as np

result_apriltag_bright = np.array([[-0.18057872276759857, -0.10362075346486935, 0.14624765585761795, 0.6442997663354058,
                                    -0.748080427360186, 0.15414119726801878, -0.038651993551756074],
                                   [-0.19423033363784747, -0.1657671879668978, -0.07876177291413694,
                                    0.39154847465927756, -0.8929749348632103, -0.1332710102115659,
                                    -0.17755110682625164],
                                   [-0.1647732608356358, -0.1576523929581787, -0.0248671537007015, 0.44628864762775894,
                                    -0.8764023952244895, -0.0622356777419214, -0.16991764198978185],
                                   [-0.20604814247362013, -0.04163313872395308, 0.035600161287915516, 0.375859462697768,
                                    -0.902745106728224, 0.006853184181097427, -0.20912668515847668],
                                   [-0.28341748262509003, -0.11821246744117975, 0.07227371793758688,
                                    0.49828203900629303, -0.8489131356148469, 0.0932729278253297, -0.14953815138348636],
                                   [-0.3058742364501822, -0.12816673083103786, 0.05834320521125827, 0.5031622194946341,
                                    -0.8374510097892137, 0.10483471902824326, -0.18577747108443812],
                                   [-0.3457300397252762, -0.17153209756790352, 0.015726055527703373, 0.5423203390934151,
                                    -0.8162423579414362, 0.051225215735119956, -0.1923877339663271],
                                   [-0.4142434184835043, -0.24757832700581422, -0.05758703146725658, 0.4872493572412943,
                                    -0.8535759715055753, 0.00028093168216614117, -0.1843801665408961],
                                   [0.01725838511215031, -0.0604869009647152, 0.06500618439658451, 0.9850064502592157,
                                    -0.15817178665248657, 0.06264853726529873, 0.028620615531779664],
                                   [-0.7023259184560533, -0.2690521224017329, -0.003998636414433293, 0.6057074293614013,
                                    -0.7731214675741853, 0.15880773834722073, -0.10090494850790002]])

result_aruco_bright = np.array([[-0.6140452871624138, -0.1872960008973478, -0.09361505881987581, 0.5459613149273779,
                                 -0.8143460515444544, 0.05448105516833823, -0.1892050886325987],
                                [-0.43582616107821154, -0.24526803856637625, -0.04674759174275503, 0.6221523031910058,
                                 -0.7687494501385123, -0.05987023706110217, -0.13552250462601834],
                                [-0.5108125838406873, -0.30169240448692236, -0.14677767706513503, 0.4971426367772406,
                                 -0.8283861074075842, -0.048812478820180805, -0.2534620241056088],
                                [-0.665884932861606, -0.34197386300218524, -0.1401128524745598, 0.5171280386927187,
                                 -0.8319133924889794, 0.01709713523934142, -0.20051530356036876],
                                [-0.5835811963313551, -0.28892792070326445, -0.022524507919000764, 0.49900113234872645,
                                 -0.8334355533003734, 0.029435499730356196, -0.23561960819340413],
                                [-0.5170899987165316, -0.26089419985112455, 0.028673637249654216, 0.4729542436753843,
                                 -0.851676260612949, 0.008740973533866936, -0.22557798181442573],
                                [-0.6776091173669327, -0.3304397335485564, -0.11132699270757992, 0.5208989216154217,
                                 -0.8288090263492203, 0.02449749544912816, -0.2028294456400942],
                                [-0.3978208253983919, -0.3084714156170073, -0.023545802733995425, 0.6525301443635796,
                                 -0.7399349980978643, -0.07828920352742874, -0.1434273680222860],
                                [-0.587855572065733, -0.27586806856172225, -0.10337328645845882, 0.5314945064388081,
                                 -0.8229139073893483, 0.03958254759255653, -0.1968743573364641],
                                [-0.7166175153464637, -0.33505974301806785, -0.17298702808894328, 0.554798478965405,
                                 -0.8090231207613447, 0.03824298063919107, -0.19030951695350914]])

result_apriltag_dim = np.array([[-0.39224833901674044, -0.07030693515289413, -0.11442264760832083, 0.6910104506529423, 0.6590193357337352, -0.025628277614129003, 0.29587372915488275],
                                [-0.377974095528978, -0.03932768283619896, -0.08822162873571136, 0.6413054784160188, 0.7386043181083473, 0.04542909135810802, 0.20279827978481832],
                                [-0.6423774495510914, -0.051487937982562876, 0.011754944925634891, 0.38209204962802595, 0.902456434920529, -0.025348690497857762, 0.19732078595937913],
                                [-0.07721821289853895, -0.06718143739556194, -0.16781006922869782, 0.8111818748542632, 0.5283622198843587, 0.17142379374585204, 0.18284204506779744],
                                [-0.50497928483461, -0.005519229979854226, 0.07610366764457424, 0.4610278641232734, 0.8666150698754262, -0.05476072980665986, 0.18284663419867622],
                                [0.03545878451561059, -0.30606505836710773, -0.2328491145185981, 0.8539725987521967, 0.3744859676463137, 0.3461537076458784, 0.10328926032960008],
                                [-0.5543025521350242, 0.034901249969003734, 0.004461179198984297, 0.5248584836089164, 0.8312893296604293, 0.023444605407377805, 0.18147168664492877],
                                [-0.42563454826423025, 0.049643748149515984, 0.056774464251890755, 0.4865034606624468, 0.8725032026373025, 0.0033252168696261686, 0.045182818458825914],
                                [-0.5004231197811717, 0.02832782190124905, 0.010532201816802027, 0.5304704652803964, 0.830737672671819, 0.09241255261863979, 0.14119463441475566],
                                [-0.5647612207731639, -0.023131769648702255, 0.046878036984599164, 0.3510932160847447, 0.9237142947536942, -0.010085470026606091, 0.15291742406049913]])

result_aruco_dim = np.array([[-0.21909491214018473, 0.03571722248281379, -0.007015662571083612, 0.6258302998602001, 0.7689877873098644, -0.12256323694176852, 0.04441251732931522],
                             [-0.3479041430341967, -0.0012919510881601176, 0.018653383441815163, 0.45130055619155696, 0.8839689375187174, -0.08029496010656328, 0.0920839012244088],
                             [-0.6136500475515193, -0.0101548175093749, 0.10278962739146405, 0.35567541864775426, 0.9227726829178478, -0.0111300978264805, 0.14785700237971458],
                             [-0.5261064266136091, -0.010053790944566321, 0.06299289967233947, 0.3582670726136193, 0.9229611790362859, -0.053342507481767276, 0.13016122144622758],
                             [-0.55653403405044, -0.035334898397995106, 0.059886328144234106, 0.39223167314909246, 0.9102612261935301, -0.0261392794708169, 0.12998289401325488],
                             [-0.5772545333757039, 0.08539191864080907, 0.09890422992723635, 0.3330626920703941, 0.9312355156882081, -0.10286498123709237, 0.10624713222874085],
                             [-0.6186749206707111, -0.0011118303484838055, 0.06869867000501922, 0.3918365261009866, 0.9013777294818454, -0.0923857002618132, 0.15952181040346236],
                             [-0.45500406482705913, 0.17513378065089275, 0.10999093606895818, 0.4522985472958965, 0.8830272144117227, -0.12484870437130519, -0.010087801499906636],
                             [-0.5918392737651075, 0.138559757530821, 0.15679017194876288, 0.4292216608853459, 0.8804424557712341, -0.10288291671153066, 0.17321937926015024],
                             [-0.6174435386083961, 0.113386055737243, 0.12480448198667143, 0.4007033897032768, 0.9155087433623117, -0.028664322092838868, 0.021421740041896842]])

result_apriltag_bright_mean = result_apriltag_bright.mean(axis=0)
result_apriltag_bright_var = result_apriltag_bright.var(axis=0)
result_aruco_bright_mean = result_aruco_bright.mean(axis=0)
result_aruco_bright_var = result_aruco_bright.var(axis=0)
print("Bright surrounding:")
print(f"The mean of apriltag result is:\n{result_apriltag_bright_mean}")
print(f"The variance of apriltag result is:\n{result_apriltag_bright_var}")
print(f"The mean of aruco result is:\n{result_aruco_bright_mean}")
print(f"The variance of aruco result is:\n{result_aruco_bright_var}")

result_apriltag_dim_mean = result_apriltag_dim.mean(axis=0)
result_apriltag_dim_var = result_apriltag_dim.var(axis=0)
result_aruco_dim_mean = result_aruco_dim.mean(axis=0)
result_aruco_dim_var = result_aruco_dim.var(axis=0)
print("Dim surrounding")
print(f"The mean of apriltag result is:\n{result_apriltag_dim_mean}")
print(f"The variance of apriltag result is:\n{result_apriltag_dim_var}")
print(f"The mean of aruco result is:\n{result_aruco_dim_mean}")
print(f"The variance of aruco result is:\n{result_aruco_dim_var}")
