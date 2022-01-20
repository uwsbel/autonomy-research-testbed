from numpy.core.numeric import NaN
import torch
import torchvision
import torch.nn as nn
import torch.optim as optim
import torchvision.transforms as transforms

# import time
import os
import glob
import numpy as np
from PIL import Image
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy import stats 
from sklearn.neighbors import NearestNeighbors

from recognition_network import *
from loader import *


per_cone_metric_names = ["GT Class", "Predicted Class", "Per Object IOU", "Normalized Size X Error", 
        "Normalized Size Y Error", "Normalized X Position Error", "Normalized Y Position Error",
        "Normalized Underprediction Amount", "Normalized Overprediction Amount", "X Position", "Y Position", "X Size", "Y Size"]
per_img_metric_names = ["Per Class Img IOU"]

class MetricComparitor():
    def __init__(self,model_name,set_a_name,set_b_name,set_a_path,set_b_path,distance_metric="Wasserstein"):
        
        self.model_name = model_name
        self.a_name = set_a_name
        self.b_name = set_b_name
        self.a_path = set_a_path
        self.b_path = set_b_path

        #load the metrics
        # self.a_img_data = []
        self.a_object_data = []
        # self.b_img_data = []
        self.b_object_data = [] 

        self.LoadMetricsFiles()
    
    def LoadMetricsFiles(self):

        # per_image_name = "per_image_metrics.csv"
        per_object_name = "per_object_metrics.csv"

        check_paths = [self.a_path,self.b_path]

        for base in [self.a_path,self.b_path]:
            # for f in [per_image_name,per_object_name]:
            file_name = os.path.join(base,per_object_name)
            check_paths.append(file_name)

        for f in check_paths:
            if(not os.path.exists(f)):
                print("Error, could not find: {}".format(f))
                exit(1)

        # self.a_img_data = np.loadtxt(os.path.join(self.a_path,"per_image_metrics.csv"))
        self.a_object_data = np.loadtxt(os.path.join(self.a_path,"per_object_metrics.csv"))
        # self.b_img_data = np.loadtxt(os.path.join(self.b_path,"per_image_metrics.csv"))
        self.b_object_data = np.loadtxt(os.path.join(self.b_path,"per_object_metrics.csv"))

    def RunConeSimilarityMetric(self):

        # run the metric for each class separately
        for c in [1,2]:
            a_subset = self.a_object_data[self.a_object_data[:,0]==c,:]
            b_subset = self.b_object_data[self.b_object_data[:,0]==c,:]

            #get feature sets
            a_iou = a_subset[:,2]
            b_iou = b_subset[:,2]

            #get feature sets
            a_features = a_subset[:,9:13]
            b_features = b_subset[:,9:13]

            print("Max A feature data:",np.max(a_features,axis=0))
            print("Min A feature data:",np.min(a_features,axis=0))
            print("Cones in reference set:",a_features.shape)

            num_neighbors = 50

            a_feats = NearestNeighbors(n_neighbors=num_neighbors, algorithm='kd_tree').fit(a_features)
            b_feats = NearestNeighbors(n_neighbors=num_neighbors, algorithm='kd_tree').fit(b_features)

            #find nearest cone in reference set
            results = []
            for i in range(b_features.shape[0]):
                b_cone_features = b_features[i,:].reshape(1,-1) 

                a_dists,a_ids = a_feats.kneighbors(b_cone_features)
                b_dists,b_ids = b_feats.kneighbors(b_cone_features)

                #do W1 measure on accuracy 
                #TODO: weight by similarity between the feature distributions?
                a_minibatch_iou = a_iou[a_ids][0]
                b_minibatch_iou = b_iou[b_ids][0]


                a_cone_sizes = a_features[a_ids,2]*a_features[a_ids,3]
                b_cone_sizes = b_features[b_ids,2]*b_features[b_ids,3]

                # print(a_cone_sizes)
                # print(b_cone_sizes)
                # exit(1)
                minibatch_similarity = stats.wasserstein_distance(a_minibatch_iou,b_minibatch_iou)
                size_similarity = stats.wasserstein_distance(a_cone_sizes[0],b_cone_sizes[0])

                mean_a_size = np.mean(a_cone_sizes)
                mean_b_size = np.mean(b_cone_sizes)

                # print("B Cone features:",b_cone_features)
                # print("Nearest A Cone features:",a_features[ids,:])
                # print("Dist={}, ID={}".format(dists,ids))

                # print("B IOU =",b_iou[i])
                # print("A IOU =",a_iou[ids])

                # mse += (b_iou[i] - np.mean(a_iou[ids]))**2
                # mae += abs(b_iou[i] - np.mean(a_iou[ids]))

                results.append([np.mean(b_iou[i] - a_iou[a_ids]),np.mean(a_dists), minibatch_similarity, size_similarity, mean_a_size, mean_b_size])
                # print("MSE =",mse)
                # print("MAE =",mae)
                # exit(1)

            results = np.asarray(results)
            print(results.shape)

            mse = np.mean(results[:,0]**2)
            mae = np.mean(np.abs(results[:,0]))

            print("MSE:",mse)
            print("MAE:",mae)
            print("Mean W1 Minibatch:",np.mean(results[:,2]))
            print("Mean W1 Minibatch size:",np.mean(results[:,3]))

            print("Mean distance to reference cone:",np.mean(results[:,1]))
            print("Max distance to reference cone:",np.max(results[:,1]))

            plt.figure()
            plt.scatter(results[:,3],results[:,2])
            plt.title("W1 IOU Minibatch vs W1 Sizes Minibatch")
            
            plt.figure()
            plt.scatter(results[:,5],results[:,4])
            plt.title("Mean A size vs mean B size")
            
            plt.figure()
            plt.scatter(results[:,5],results[:,3])
            plt.title("W1 Size Minibatch vs mean B size")

            plt.figure()
            plt.scatter(results[:,4],results[:,3])
            plt.title("W1 Size Minibatch vs mean A size")

            plt.figure()
            plt.scatter(results[:,5],results[:,2])
            plt.title("W1 IOU Minibatch vs Mean A Size")
            
            plt.figure()
            plt.scatter(results[:,4],results[:,2])
            plt.title("W1 IOU Minibatch vs Mean B Size")
            
            

            # plt.figure()
            # plt.hist(results[:,0])3
            
            # plt.figure()
            # plt.hist(results[:,1])
            # plt.title("Distance along features to nearest cone in reference dataset. \nLarge values mean the compared cone was not similar.")

            plt.show()

        exit(1)


    def GenerateComparisonPlots(self,save=False,display=False,output_path="output_compare"):    

        if(not os.path.exists(output_path)):
            os.mkdir(output_path)
        
        # === generate per image comparison ===
        # fig, axs = plt.subplots(2,1)
        # fig.tight_layout(h_pad=1.0,rect=(0,0,1,.8))

        # wass_distances = np.zeros(self.a_img_data.shape[1])
        # mean_distances = np.zeros(self.a_img_data.shape[1])

        

        # for c in range(self.a_img_data.shape[1]):
        #     range_max = np.max([np.max(self.a_img_data[:,c]),np.max(self.b_img_data[:,c])])
        #     range_min = np.min([np.min(self.a_img_data[:,c]),np.min(self.b_img_data[:,c])])

        #     bins = 20
        #     axs[0].hist(self.a_img_data[:,c],range=(range_min,range_max),bins=bins,alpha=.5,color=colors[c])
        #     axs[0].axvline(self.a_img_data[:,c].mean(),color=colors[c-1], linestyle='dashed', linewidth=2)

        #     axs[1].hist(self.b_img_data[:,c],range=(range_min,range_max),bins=bins,alpha=.5,color=colors[c])
        #     axs[1].axvline(self.b_img_data[:,c].mean(),color=colors[c], linestyle='dashed', linewidth=2)

        #     wass_distances[c] = stats.wasserstein_distance(self.a_img_data[:,c],self.b_img_data[:,c])

        #     mean_distances[c] = np.mean(self.a_img_data[:,c]) - np.mean(self.b_img_data[:,c])

        # #set titles
        # axs[0].set_title("Data: {}".format(self.a_name))
        # axs[1].set_title("Data: {}".format(self.b_name))

        # axs[0].legend(["Red Cones", "Green Cones"])
        # axs[1].legend(["Red Cones", "Green Cones"])
        # fig.suptitle("{} \n Model={} \n Means Dist ({}-{}) Per Class = {} \n Wasserstein Distances Per Class = {}".format(
        #     per_img_metric_names[0],self.model_name,self.a_name,self.b_name,[ '%.6f' % w for w in mean_distances ],[ '%.6f' % w for w in wass_distances ]))

        # if(save):
        #     fig.savefig(os.path.join(output_path,self.model_name+"_"+per_img_metric_names[0].replace(" ","_")+".png"),dpi=300)




        colors = ['C0','C1']
        # === generate per object comparison ===
        for m in range(self.a_object_data.shape[1]):

            fig, axs = plt.subplots(2,1)
            fig.tight_layout(h_pad=1.0,rect=(0,0,1,.8))

            wass_distances = np.zeros(2)
            mean_distances = np.zeros(2)

            num_a_samples = [0,0]
            num_b_samples = [0,0]

            #for each of the two classes
            for c in [1,2]:
                a_subset = self.a_object_data[self.a_object_data[:,0]==c,m]
                b_subset = self.b_object_data[self.b_object_data[:,0]==c,m]

                                # if(m==2):
                #filter out for size (temporary to see what matters)
                # a_widths = self.a_object_data[self.a_object_data[:,0]==c,11]
                # b_widths = self.b_object_data[self.b_object_data[:,0]==c,11]
                # a_heights = self.a_object_data[self.a_object_data[:,0]==c,12]
                # b_heights = self.b_object_data[self.b_object_data[:,0]==c,12]

                # pixel_count_threshold = 225
                # a_count_before = len(a_subset)
                # b_count_before = len(b_subset)
                # a_subset = a_subset[a_widths*a_heights >= pixel_count_threshold]
                # b_subset = b_subset[b_widths*b_heights >= pixel_count_threshold]

                # a_count_after = len(a_subset)
                # b_count_after = len(b_subset)

                # print("thresholded {}/{} and {}/{}".format(a_count_before,b_count_before,a_count_after,b_count_after))


                #filter out NaN
                a_subset = a_subset[np.logical_not(np.isnan(a_subset))]
                b_subset = b_subset[np.logical_not(np.isnan(b_subset))]


                range_max = np.max([np.max(a_subset),np.max(b_subset)])
                range_min = np.min([np.min(a_subset),np.min(b_subset)])

                num_a_samples[c-1] = len(a_subset)
                num_b_samples[c-1] = len(b_subset)

                bins = int(2*np.sqrt(min(len(a_subset),len(b_subset))))

                axs[0].hist(a_subset,range=(range_min,range_max),bins=bins,alpha=.5,color=colors[c-1])
                axs[0].axvline(a_subset.mean(),color=colors[c-1], linestyle='dashed', linewidth=2)

                axs[1].hist(b_subset,range=(range_min,range_max),bins=bins,alpha=.5,color=colors[c-1])
                axs[1].axvline(b_subset.mean(),color=colors[c-1], linestyle='dashed', linewidth=2)

                wass_distances[c-1] = stats.wasserstein_distance(a_subset,b_subset)
                mean_distances[c-1] = np.mean(a_subset) - np.mean(b_subset)

            #set titles
            axs[0].set_title("Data: {}".format(self.a_name))
            axs[1].set_title("Data: {}".format(self.b_name))

            axs[0].legend(["Red Cones N={}".format(num_a_samples[0]), "Green Cones N={}".format(num_a_samples[1])])
            axs[1].legend(["Red Cones N={}".format(num_b_samples[0]), "Green Cones N={}".format(num_b_samples[1])])

            fig.suptitle("{} \n Model={} \n Means Dist ({}-{}) Per Class = {} \n Wasserstein Distances Per Class = {}".format(
                per_cone_metric_names[m],self.model_name,self.a_name,self.b_name,[ '%.6f' % w for w in mean_distances ],[ '%.6f' % w for w in wass_distances ]))
            if(save):
                fig.savefig(os.path.join(output_path,self.model_name+"_"+per_cone_metric_names[m].replace(" ","_")+".png"),dpi=300)


        #make some 2d histograms between the metrics and the features

        feature = 10
        metric = 2
        a_subset = self.a_object_data[:,metric]
        b_subset = self.b_object_data[:,metric]
        a_features = self.a_object_data[:,feature]
        b_features = self.b_object_data[:,feature]

        a_valid = np.logical_not(np.isnan(a_subset))
        b_valid = np.logical_not(np.isnan(b_subset))

        a_subset = a_subset[a_valid]
        b_subset = b_subset[b_valid]
        a_features = a_features[a_valid]
        b_features = b_features[b_valid]

        fig, axs = plt.subplots(2,1)
        fig.tight_layout(h_pad=1.0,rect=(0,0,1,.8))
        axs[0].hist2d(a_subset,a_features,bins=30)
        axs[1].hist2d(b_subset,b_features,bins=30)

        axs[0].set_xlabel(per_cone_metric_names[metric])
        axs[1].set_xlabel(per_cone_metric_names[metric])
        axs[0].set_ylabel(per_cone_metric_names[feature])
        axs[1].set_ylabel(per_cone_metric_names[feature])

        axs[0].set_title("Data: {}".format(self.a_name))
        axs[1].set_title("Data: {}".format(self.b_name))



        if(display):
            plt.show()
        plt.close('all')

        
class MetricTester():
    def __init__(self, model, loader, output_path="output", name="tester"):
        self.model = model
        self.loader = loader
        self.output_path = output_path
        self.name = name

        print(type(model))
        print(type(loader))

        # self.per_image_metrics = []
        self.per_cone_metrics = []
        self.classes = [] #classes included in the metrics

        self.w = 640 #default image size - will be updated when running test
        self.h = 360 #default image size - will be updated when running test


    def RunTest(self,classes=[1,2]):
        self.classes = classes
        self.model.eval()

        for i, data in enumerate(self.loader):
            imgs, boxes, labels = data
            self.h = imgs[0].size()[1]
            self.w = imgs[0].size()[2]

            print("Test [{}/{}]".format(i+1,len(self.loader)))

            img_list = []
            target_list = []
            for b in range(imgs.size()[0]):
                img_tensor = imgs[b, :, :, :].cuda()
                target_dict = {}
                ids = labels[b, :] > 0
                target_dict["boxes"] = boxes[b, ids, :].cuda()
                target_dict["labels"] = labels[b, ids].cuda()
                img_list.append(img_tensor)
                target_list.append(target_dict)

            # print("len(img_list):",len(img_list))

            prediction = self.model.predict(img_list)

            #DEBUG INFO
            fig, ax = plt.subplots()
            gt_boxes = target_list[0]["boxes"].cpu().detach().numpy()
            gt_labels = target_list[0]["labels"].cpu().detach().numpy()
            pred_boxes = prediction[0]["boxes"].cpu().detach().numpy()
            pred_labels = prediction[0]["labels"].cpu().detach().numpy()

            ax.imshow(img_list[0].cpu().detach().numpy().transpose((1,2,0)))
            for b in range(gt_boxes.shape[0]):    
                color = 'b'
                rect = patches.Rectangle((gt_boxes[b, 0], gt_boxes[b, 1]), gt_boxes[b, 2]-gt_boxes[b, 0], gt_boxes[b, 3]-gt_boxes[b,1], linewidth=2, edgecolor=color, facecolor='none')
                ax.add_patch(rect)
                # self.ax.text(boxes[b, 0], boxes[b, 1], "{:.2f}".format(scores[b]), fontsize=8)

            for b in range(pred_boxes.shape[0]):    
                color = 'r' if int(pred_labels[b])==1 else 'g'
                rect = patches.Rectangle((pred_boxes[b, 0], pred_boxes[b, 1]), pred_boxes[b, 2]-pred_boxes[b, 0], pred_boxes[b, 3]-pred_boxes[b,1], linewidth=2, edgecolor=color, facecolor='none')
                ax.add_patch(rect)
                # self.ax.text(boxes[b, 0], boxes[b, 1], "{:.2f}".format(scores[b]), fontsize=8)

            plt.show()

            #get the metrics for the image as a whole as well as for each cone
            # self.per_image_metrics.append(self.GetPerImgMetric(prediction[0],target_list[0]))

            img_cone_metrics = self.GetPerConeMetrics(prediction[0],target_list[0])
            for metric in img_cone_metrics:
                self.per_cone_metrics.append(metric)
            
    def CalcClassIOUPerImg(self,pred_boxes,pred_labels,gt_boxes,gt_labels,class_id):
        #make pixel masks for precition and ground truth
        gt_box_mask = np.zeros((self.w,self.h))
        pred_box_mask = np.zeros((self.w,self.h))

        for b in gt_boxes[gt_labels==class_id]:
            x0 = int(round(b[0]))
            y0 = int(round(b[1]))
            x1 = int(round(b[2]))
            y1 = int(round(b[3]))
            gt_box_mask[x0:x1,y0:y1] += 1

        for b in pred_boxes[pred_labels==class_id]:
            x0 = int(round(b[0]))
            y0 = int(round(b[1]))
            x1 = int(round(b[2]))
            y1 = int(round(b[3]))
            pred_box_mask[x0:x1,y0:y1] += 1

        intersection_mask = np.min(np.array([gt_box_mask,pred_box_mask]),axis=0)

        # print("intersection_mask.shape",intersection_mask.shape)
        
        n_intersection = np.sum(intersection_mask)
        n_pred = np.sum(pred_box_mask)
        n_gt = np.sum(gt_box_mask)
        iou = 0
        if(n_pred + n_gt > 0): #typical situation
            iou = n_intersection / (n_pred + n_gt - n_intersection)
        else: #situation where no GT objects and no prediction - perfect IOU
            iou = 1.0
        
        return iou


    # def GetPerImgMetric(self,prediction,ground_truth):
    #     #calculate total image IOU
        
    #     gt_boxes = ground_truth["boxes"].cpu().detach().numpy()
    #     gt_labels = ground_truth["labels"].cpu().detach().numpy()
    #     pred_boxes = prediction["boxes"].cpu().detach().numpy()
    #     pred_labels = prediction["labels"].cpu().detach().numpy()
        
    #     iou_per_class = []
    #     for c in self.classes:
    #         iou = self.CalcClassIOUPerImg(pred_boxes,pred_labels,gt_boxes,gt_labels,c)
    #         iou_per_class.append(iou)

    #     # print(iou_per_class)
    #     return np.asarray(iou_per_class)

    # def GetConeMetricsForClass(self,prediction,ground_truth,class_id):

    #     pass

    def GetPerConeMetrics(self,prediction,ground_truth):
        #metrics: center error (x,y), size error (x,y), IOU

        #perform box matching for each class
        gt_boxes = ground_truth["boxes"].cpu().detach().numpy()
        gt_labels = ground_truth["labels"].cpu().detach().numpy()
        pred_boxes = prediction["boxes"].cpu().detach().numpy()
        pred_labels = prediction["labels"].cpu().detach().numpy()

        # do matching on all classes together to include mislabeling in error metrics 
        gt_boxes = gt_boxes[gt_labels>0]
        gt_labels = gt_labels[gt_labels>0]
        pred_boxes = pred_boxes[pred_labels>0]
        pred_labels = pred_labels[pred_labels>0]

        # print("Predicted labels:",len(pred_labels))
        # print("GT labels:",len(gt_labels))

        iou_matrix = np.zeros((len(pred_labels),len(gt_labels)))

        #calculate iou for predicted box vs each gt box
        for p in range(len(pred_labels)):
            for g in range(len(gt_labels)):
                gx0,gy0,gx1,gy1 = gt_boxes[g,:].astype(np.int32)
                px0,py0,px1,py1 = pred_boxes[p,:].astype(np.int32)

                #do mask comparison
                mask = np.zeros((self.w,self.h)).astype(np.int32)
                mask[gx0:gx1,gy0:gy1] += 1
                mask[px0:px1,py0:py1] += 1

                intersection = np.count_nonzero(mask[mask==2])
                union = np.count_nonzero(mask[mask>0])

                iou_matrix[p,g] = intersection / float(union)
                # print("GT Box={}, Pred Box={}, Intersection = {}, Union={}, IOU={}".format(gt_boxes[g,:],pred_boxes[p,:],intersection,union,intersection / union))
                # exit(1)

        #greedily find the max IOU to match boxes
        pairs = []

        gt_ids = list(range(len(gt_labels)))
        pred_ids = list(range(len(pred_labels)))
        
        if(len(pred_labels) > 0 and len(gt_labels) > 0):
            for p in range(len(pred_labels)):
                if(np.max(iou_matrix) < 1e-9):
                    break

                id_max = np.unravel_index(iou_matrix.argmax(), iou_matrix.shape)
                pairs.append(id_max)
                
                pred_ids.remove(id_max[0])
                gt_ids.remove(id_max[1])

                # print("Max IOU remaining:",iou_matrix[id_max])
                # print("iou_matrix.shape",iou_matrix.shape)

                #zero out the row and column selected
                iou_matrix[id_max[0],:] = -1
                iou_matrix[:,id_max[1]] = -1

        #handle any remaining predictions or gt boxes with no pairs
        for p in pred_ids:
            pairs.append((p,-1))
        for g in gt_ids:
            pairs.append((-1,g))      

        # print("iou_matrix.shape",iou_matrix.shape)
        # print("pairs",pairs)

        per_cone_metrics = []
        for p,pair in enumerate(pairs):
            p_id = pair[0]
            g_id = pair[1]

            #=== calculate errors and metrics

            #default metric values
            g_class = 0
            p_class = 0
            object_iou = 0
            sx_error = np.NaN
            sy_error = np.NaN
            x_error = np.NaN
            y_error = np.NaN
            under = np.NaN
            over = np.NaN

            if(p_id == -1):
                g_class = gt_labels[g_id].astype(np.int32)

            elif(g_id == -1):
                p_class = pred_labels[p_id].astype(np.int32)

            else:
                gx0,gy0,gx1,gy1 = gt_boxes[g_id,:].astype(np.int32)
                g_class = gt_labels[g_id].astype(np.int32)
                px0,py0,px1,py1 = pred_boxes[p_id,:].astype(np.int32)
                p_class = pred_labels[p_id].astype(np.int32)

            
                #IOU
                mask = np.zeros((self.w,self.h)).astype(np.int32)
                mask[gx0:gx1,gy0:gy1] += 1
                mask[px0:px1,py0:py1] += 1

                intersection = np.count_nonzero(mask[mask==2])
                union = np.count_nonzero(mask[mask>0])
                object_iou = intersection / union

                #normalized x size error
                sx_error = ((px1-px0) - (gx1-gx0)) / (gx1-gx0)

                #normalized y size error
                sy_error = ((py1-py0) - (gy1-gy0)) / (gy1-gy0)

                #signed and normalized x position error
                p_x_pos = (px1+px0) / 2.0
                g_x_pos = (gx1+gx0) / 2.0
                x_error = (p_x_pos - g_x_pos) / (gx1-gx0)

                #signed y position error
                p_y_pos = (py1+py0) / 2.0
                g_y_pos = (gy1+gy0) / 2.0
                y_error = (p_y_pos - g_y_pos) / (gy1-gy0)

                #underpredicted percent
                gt_pixel_count = (gy1-gy0)*(gx1-gx0)
                under = (gt_pixel_count - intersection) / float(gt_pixel_count)

                #overpredicted percent
                pred_pixel_count = (py1-py0)*(px1-px0)
                over = (pred_pixel_count - intersection) / float(pred_pixel_count)


            #engineering features of interest

            #valid if ground truth exists
            if(g_id != -1):
                gx0,gy0,gx1,gy1 = gt_boxes[g_id,:].astype(np.int32)
                x_pos = ((gx1+gx0) / 2 - (self.w / 2)) / (self.w / 2)
                y_pos = ((gy1+gy0) / 2 - (self.h / 2)) / (self.h / 2)

                x_size = gx1 - gx0
                y_size = gy1 - gy0
                
            
            # version for phantom predictions
            else:
                px0,py0,px1,py1 = pred_boxes[p_id,:].astype(np.int32)
                x_pos = ((px1+px0) / 2 - (self.w / 2)) / (self.w / 2)
                y_pos = ((py1+py0) / 2 - (self.h / 2)) / (self.h / 2)

                x_size = px1 - px0
                y_size = py1 - py0
                


                #TODO: other useful per-cone metrics

            per_cone_metrics.append([g_class,p_class,object_iou,sx_error,sy_error,x_error,y_error,under,over,x_pos,y_pos,x_size,y_size])


        # fig, ax = plt.subplots()
        # ax.imshow(255*np.ones((720,1280,3)))
        # for b in range(len(gt_labels)):
        #     rect = patches.Rectangle(
        #         (gt_boxes[b, 0], gt_boxes[b, 1]), gt_boxes[b, 2]-gt_boxes[b, 0], gt_boxes[b, 3]-gt_boxes[b, 1], linewidth=1, edgecolor='b', facecolor='none')
        #     ax.add_patch(rect)
        # for b in range(len(pred_labels)):
        #     rect = patches.Rectangle(
        #         (pred_boxes[b, 0], pred_boxes[b, 1]), pred_boxes[b, 2]-pred_boxes[b, 0], pred_boxes[b, 3]-pred_boxes[b, 1], linewidth=1, edgecolor='r', facecolor='none')
        #     ax.add_patch(rect)

        # plt.show()
        # exit(1)

        return per_cone_metrics


    def SaveMetrics(self,display=False):
        if(not os.path.exists(self.output_path)):
            os.mkdir(self.output_path)

        # iou_per_img_per_class = np.asarray(self.per_image_metrics)
        cone_metrics = np.asarray(self.per_cone_metrics)

        #save metrics to a file
        # np.savetxt(os.path.join(self.output_path,"per_image_metrics.csv"),iou_per_img_per_class)
        np.savetxt(os.path.join(self.output_path,"per_object_metrics.csv"),cone_metrics)

        #display histogram of metrics if requested
        if(display):
            # #PER IMAGE METRICS
            # fig = plt.figure()
            # bins = int(np.round(2*np.sqrt(iou_per_img_per_class.shape[0])))
            # for i in range(iou_per_img_per_class.shape[1]):
            #     # print("bins=",bins,"iou_per_img_per_class.shape[1]",iou_per_img_per_class.shape[0])
            #     plt.hist(iou_per_img_per_class[:,i],alpha=1 / np.sqrt(iou_per_img_per_class.shape[1]),bins=bins)
            # plt.legend(self.classes)
            # plt.title("Per Image IOU")

            #PER CONE METRICS
            bins = int(np.round(2*np.sqrt(cone_metrics.shape[0])))
            # === List Of Errors and Metrics ===

            # loop through metrics
            for i in range(1,cone_metrics.shape[1]):
                fig = plt.figure()
                for c in self.classes:
                    metrics = cone_metrics[cone_metrics[:,0]==c]
                    plt.hist(metrics[:,i],alpha=1 / len(self.classes),bins=bins)
                plt.legend(self.classes)
                plt.title(per_cone_metric_names[i])
                
            plt.show()