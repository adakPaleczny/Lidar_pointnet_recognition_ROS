# ==============================================================================
# -- Global package ------------------------------------------------------------
# ==============================================================================
import numpy as np
import tensorflow as tf
from tensorflow import keras
from tensorflow.keras.models import Model
from tensorflow.keras.models import load_model
from dv_interfaces.msg import ClusterArray

# ==============================================================================
# -- ROS package ---------------------------------------------------------------
# ==============================================================================
# from sensor_msgs.point_cloud2 import PointCloud2 
import rospy
# import pcl
import ctypes
import struct
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


from tensorflow.keras import layers

import numpy as np
import random

def conv_bn(x, filters):
  x = layers.Conv1D(filters, kernel_size=1, padding='valid')(x)
  x = layers.BatchNormalization(momentum=0.0)(x)
  return layers.Activation('relu')(x)

def dense_bn(x, filters):
  x = layers.Dense(filters)(x)
  x = layers.BatchNormalization(momentum=0.0)(x)
  return layers.Activation('relu')(x)


class OrthogonalRegularizer(keras.regularizers.Regularizer):
  def __init__(self, num_features, l2reg=0.001):
    self.num_features = num_features
    self.l2reg = l2reg
    self.eye = tf.eye(num_features)

  def __call__(self, x):
    x = tf.reshape(x, (-1, self.num_features, self.num_features))
    xxt = tf.tensordot(x, x, axes=(2,2))
    xxt = tf.reshape(xxt, (-1, self.num_features, self.num_features))
    return tf.reduce_sum(self.l2reg * tf.square(xxt - self.eye))


def tnet(inputs, num_features):
    # initialize bias as the identity matrix
    bias = keras.initializers.Constant(np.eye(num_features).flatten())
    reg = OrthogonalRegularizer(num_features)

    x = conv_bn(inputs, 32)
    x = conv_bn(x, 64)
    x = conv_bn(x, 512)
    x = layers.GlobalMaxPooling1D()(x)
    x = dense_bn(x, 256)
    x = dense_bn(x, 128)
    x = layers.Dense(
        num_features * num_features,
        kernel_initializer='zeros',
        bias_initializer=bias,
        activity_regularizer=reg
    )(x)
    feat_T = layers.Reshape((num_features, num_features))(x)
    # Apply affine transformation to input features
    return layers.Dot(axes=(2, 1))([inputs, feat_T])

class PointNet_Model():
    def __init__(self, N_POINTS, N_CLASSES) -> None:
        inputs = keras.Input(shape=(N_POINTS, 3))

        x = tnet(inputs, 3)
        x = conv_bn(x, 32)
        x = conv_bn(x, 32)
        x = tnet(x, 32)
        x = conv_bn(x, 32)
        x = conv_bn(x, 64)
        x = conv_bn(x, 512)
        x = layers.GlobalMaxPooling1D()(x)
        x = dense_bn(x, 256)
        x = layers.Dropout(0.3)(x)
        x = dense_bn(x, 128)
        x = layers.Dropout(0.3)(x)

        outputs = layers.Dense(N_CLASSES, activation='sigmoid')(x)

        self.model = keras.Model(inputs=inputs, outputs=outputs, name='pointnet')

        self.model.load_weights('pn_weights.h5')

class PointNet:
    def __init__(self) -> None:
        # self.session = tf.Session()
        self.print = True
        # try:
        self.pointnet = PointNet_Model(25,1)
            # rospy.loginfo("MODEL LOADED")
        # except:
        #     rospy.loginfo("SOME EXEPTION WAS CAUGHT WITH LOADING POINTNET MODEL")

        # self.sub = rospy.Subscriber("euclidean_clustering/clusters", ClusterArray, self.callback)
        # self.pub = rospy.Publisher("pointnet_cones", PointCloud2 ,queue_size=10)
        
    def combine_clusters(self, msg):
        # Initialize an empty list to hold all points
        all_points = []

        # Iterate over clusters
        for cluster in msg.clusters:
            # Convert each cluster to a list of points
            cluster_points = pc2.read_points(cluster, field_names=("x", "y", "z"), skip_nans=True)
            all_points.extend(cluster_points)

        # Create a header for the combined point cloud
        header = msg.header
        header.frame_id = 'os_sensor'
        # Create the combined point cloud
        combined = pc2.create_cloud_xyz32(header, all_points)

        return combined
    
    def callback(self, ros_point_cloud):
        time = rospy.get_time()
        msg = ClusterArray()
        
        for cluster in ros_point_cloud.clusters:
            xyz = np.array([[0,0,0]])
            rgb = np.array([[0,0,0]])
            #self.lock.acquire()
            gen = pc2.read_points(cluster, skip_nans=True)
            int_data = list(gen)

            for x in int_data:
                test = x[3] 
                # cast float32 to int so that bitwise operations are possible
                s = struct.pack('>f' ,test)
                i = struct.unpack('>l',s)[0]
                # you can get back the float value by the inverse operations
                pack = ctypes.c_uint32(i).value
                r = (pack & 0x00FF0000)>> 16
                g = (pack & 0x0000FF00)>> 8
                b = (pack & 0x000000FF)
                # prints r,g,b values in the 0-255 range
                            # x,y,z can be retrieved from the x[0],x[1],x[2]
                xyz = np.append(xyz,[[x[0],x[1],x[2]]], axis = 0)
                rgb = np.append(rgb,[[r,g,b]], axis = 0)
            
            points = []
            data = []

            for x in xyz:
                data.append(list(x))

            if len(xyz) > 24:
                for i in range(25):
                    ran = random.choice(data)
                    points.append(ran)
                points = np.reshape(points, (1,25,3))
                info = self.pointnet.model.predict(points, verbose="0")
                if info > 0.4:
                    msg.clusters.append(cluster)

        pc_msg = self.combine_clusters(msg)
        self.pub.publish(pc_msg)        
        rospy.loginfo_throttle(10, f'[POINTNET MODEL] DURATION: {(rospy.get_time()-time)*1000:.2f} ms')


def main():
    rospy.init_node("pointnet")

    try:
        PointNet()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__ == '__main__':
    main()