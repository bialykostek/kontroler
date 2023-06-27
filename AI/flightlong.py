#run with condra (virtualenv):
#conda activate tf
#tensorflow(-gpu)==2.1
#python 3.7.9

#https://colab.research.google.com/github/tensorflow/tensorflow/blob/master/tensorflow/lite/examples/experimental_new_converter/Keras_LSTM_fusion_Codelab.ipynb#scrollTo=0-b0IKK2FGuO

import os
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import math
from tensorflow.keras import layers

EPOCHS = 500
BATCH_SIZE = 100

val_ratio = 0.2
test_ratio = 0.2

tflite_model_name = "fligh_model"
c_model_name = "fligh_model"

file = open('log_21_06_13_27_44.txt' , 'r')

inputdata = []
inputres = []

recc=2
skip = 0

while True:
    line = file.readline()
    
    if not line:
        break
  
    try:

      line = line.split(",")
      line = line[33]

      line = line.split("|")[0]
      line = line.split(">")

      tmp = []
      for i in range(19):
        tmp.append(float(line[i]))
      
      if skip < recc:
          skip += 1
          print("skippin")
      else:
        if len(inputdata[len(inputdata)-1]) == 19 or len(inputdata[len(inputdata)-1]) == 38:
            for i in range(0, len(inputdata[len(inputdata)-1])):
                tmp.append(inputdata[len(inputdata)-1][i])
        else:
            for i in range(0, len(inputdata[len(inputdata)-1])-19):
                tmp.append(inputdata[len(inputdata)-1][i])

      print(tmp)
      inputdata.append(tmp)
      inputres.append([float(line[19]), float(line[20]), float(line[21]), float(line[22]), float(line[23])])
    except:
      pass


#x_values = np.array(inputdata).reshape(len(inputdata), 1, 21)
x_values = np.array(inputdata)
y_values = np.array(inputres)

val_split = int(val_ratio * len(x_values))
test_split = int(val_split + (test_ratio * len(x_values)))
x_val, x_test, x_train = np.split(x_values, [val_split, test_split])
y_val, y_test, y_train = np.split(y_values, [val_split, test_split])

print(x_train.shape)
x_train= np.reshape(x_train,(x_train.shape[0], 1, x_train.shape[1]))


model = tf.keras.Sequential()
model.add(layers.LSTM(22, activation='sigmoid', input_shape=x_train.shape, return_sequences=True))
#model.add(layers.Dense(22, activation='sigmoid', input_shape=(21,)))
model.add(layers.Dense(15, activation='sigmoid'))
model.add(layers.Dense(5))

model.compile(optimizer='rmsprop', loss='mae', metrics=['mae'])

history = model.fit(x_train, y_train, epochs=EPOCHS, batch_size=BATCH_SIZE, validation_data=(x_val, y_val))


loss = history.history['loss']
val_loss = history.history['val_loss']

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()

open(tflite_model_name + '.tflite', 'wb').write(tflite_model)

def hex_to_c_array(hex_data, var_name):
  c_str = ''
  c_str += '#ifndef ' + var_name.upper() + '_H\n'
  c_str += '#define ' + var_name.upper() + '_H\n\n'
  c_str += '\nunsigned int ' + var_name + '_len = ' + str(len(hex_data)) + ';\n'
  c_str += 'unsigned char ' + var_name + '[] = {'
  hex_array = []
  for i, val in enumerate(hex_data) :
    hex_str = format(val, '#04x')
    if (i + 1) < len(hex_data):
      hex_str += ','
    if (i + 1) % 12 == 0:
      hex_str += '\n '
    hex_array.append(hex_str)
  c_str += '\n ' + format(' '.join(hex_array)) + '\n};\n\n'
  c_str += '#endif //' + var_name.upper() + '_H'
  return c_str

with open(c_model_name + '.h', 'w') as file:
  file.write(hex_to_c_array(tflite_model, c_model_name))

x = inputdata[0]
y = inputres[0]
print(x)
print(y)
print(model.predict([x]))


#model.add(layers.LSTM(22, activation='sigmoid', input_shape=x_train.shape[1:], return_sequences=True))
