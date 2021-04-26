#!/usr/bin/env python

import numpy as np
import tensorflow as tf

import cv2
import os


camera = cv2.VideoCapture(0)
#camera = cv2.imread(r'images/cv_imagec8.png',1)
def load_graph(model_file):
  graph = tf.Graph()
  graph_def = tf.GraphDef()
  #graph_def = tf.compat.v1.GraphDef() #if tf version 2

  with open(model_file, "rb") as f:
    graph_def.ParseFromString(f.read())
  with graph.as_default():
    tf.import_graph_def(graph_def)

  return graph



def load_labels(label_file):
  label = []
  proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
  #proto_as_ascii_lines = tf.compat.v1.gfile.GFile(label_file).readlines() #tf version 2
  for l in proto_as_ascii_lines:
    label.append(l.rstrip())
  return label

def grabVideoFeed():
    #grabbed, frame = camera.read()
    #return frame if grabbed else None
    frame = camera
    return frame


if __name__ == "__main__":

    #path = os.path.expanduser("~/catkin_ws/src/group_project/project/src/final_graph.pb")

    model_file = os.path.expanduser("~/catkin_ws/src/group_project/project/src/final_graph.pb")
    label_file = os.path.expanduser("~/catkin_ws/src/group_project/project/src/final_graph.pb")
    input_height = 224
    input_width = 224
    input_mean = 0
    input_std = 255
    input_layer = "Placeholder"
    output_layer = "final_result"

    #load tensorflow graph and labels file
    graph = load_graph(model_file)
    labels = load_labels(label_file)

    #set the inputs and outputs for the graph
    input_name = "import/" + input_layer
    output_name = "import/" + output_layer
    input_operation = graph.get_operation_by_name(input_name)
    output_operation = graph.get_operation_by_name(output_name)
    #with tf.compat.v1.Session(graph=graph) as sess: #tf version 2
    with tf.Session(graph=graph) as sess:
        while True:

            live_frame = grabVideoFeed()
            if live_frame is None:
                raise SystemError('Issue grabbing the frame')

            resized_frame = cv2.resize(live_frame, (input_height,input_width), interpolation=cv2.INTER_CUBIC)
            cv2.imshow("Image", resized_frame)

            numpy_frame = np.float32(resized_frame)
            #ensure frame have same mean and range
            normalised = cv2.normalize(numpy_frame, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
            t = np.expand_dims(normalised, axis=0)

            #run forward pass of framne through net
            results = sess.run(output_operation.outputs[0], {
                               input_operation.outputs[0]: t})
            #find the highest output
            results = np.squeeze(results)
            top_k = results.argsort()[-7:][::-1]
            print(labels[top_k[0]], results[top_k[0]])

            if cv2.waitKey(1) & 0xFF == ord('q'):
                sess.close()
                break
            break

    #camera.release()
    cv2.destroyAllWindows()
