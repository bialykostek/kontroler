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

EPOCHS = 300
BATCH_SIZE = 32

val_ratio = 0.2
test_ratio = 0.05

tflite_model_name = "flight_model"
c_model_name = "flight_model"

file = open('log_21_06_17_17_21.txt' , 'r')

inputdata = []
inputres = []
s=0
while True:
    line = file.readline() 

    s+=1
    if s<269:
      continue
    if s > 6888:
      break
       
    
    if not line:
        break
  
    try:

      line = line.split(",")
      line = line[33]

      line = line.split("|")[0]
      line = line.split(">")
      
      ok = True
      for i in range(21):
        if math.isnan(float(line[i])):
          ok = False
          break
      if not ok:
        continue

      tmp = []
      #!!!16
      for i in range(16):
        tmp.append(float(line[i]))
  
      inputres.append([float(line[21]), float(line[22]), float(line[23]), float(line[24]), float(line[25])])
      inputdata.append(tmp)
    except:
      pass

x_values = np.array(inputdata)
y_values = np.array(inputres)

val_split = int(val_ratio * len(x_values))
test_split = int(val_split + (test_ratio * len(x_values)))
x_val, x_test, x_train = np.split(x_values, [val_split, test_split])
y_val, y_test, y_train = np.split(y_values, [val_split, test_split])

print(x_train.shape)
print(y_train.shape)


model = tf.keras.Sequential()
#!!!!16
model = tf.keras.Sequential()
model.add(layers.Dense(30, activation='relu', input_shape=(16,)))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(5))

#model.add(layers.LSTM(22, activation='sigmoid', input_shape=x_train.shape, return_sequences=True))
#model.optimizer.learning_rate = 0.01


model.compile(optimizer=tf.keras.optimizers.Adam(learning_rate=1e-3), loss=tf.keras.losses.MeanSquaredError())

es_callback = tf.keras.callbacks.EarlyStopping(monitor='val_loss', patience=15)

history = model.fit(x_values, y_values, epochs=EPOCHS, batch_size=BATCH_SIZE, validation_data=(x_val, y_val), callbacks=[es_callback])


#loss = history.history['loss']
#val_loss = history.history['val_loss']

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()

open(tflite_model_name + '.tflite', 'wb').write(tflite_model)

def hex_to_c_array(hex_data, var_name):
  c_str = """
#define TENSORFLOW_LITE_EXPERIMENTAL_MICRO_EXAMPLES_HELLO_WORLD_SINE_MODEL_DATA_H_
#ifdef __has_attribute
#define HAVE_ATTRIBUTE(x) __has_attribute(x)
#else
#define HAVE_ATTRIBUTE(x) 0
#endif
#if HAVE_ATTRIBUTE(aligned) || (defined(__GNUC__) && !defined(__clang__))
#define DATA_ALIGN_ATTRIBUTE __attribute__((aligned(4)))
#else
#define DATA_ALIGN_ATTRIBUTE
#endif
"""
  c_str += '\nconst int ' + var_name + '_len = ' + str(len(hex_data)) + ';\n'
  c_str += 'const unsigned char ' + var_name + '[] DATA_ALIGN_ATTRIBUTE = {'
  hex_array = []
  for i, val in enumerate(hex_data) :
    hex_str = format(val, '#04x')
    if (i + 1) < len(hex_data):
      hex_str += ','
    if (i + 1) % 12 == 0:
      hex_str += '\n '
    hex_array.append(hex_str)
  c_str += '\n ' + format(' '.join(hex_array)) + '\n};\n\n'

  return c_str

with open(c_model_name + '.h', 'w') as file:
  file.write(hex_to_c_array(tflite_model, c_model_name))

'''
output = []
for i in range(len(x_test)-1):
  print(str(i) + "/"+str(len(x_test)))
  out = model.predict(x_test[i:i+1])
  output.append(out[0].tolist())
  x_test[i+1][16] = out[0][0]
  x_test[i+1][17] = out[0][1]
  x_test[i+1][18] = out[0][2]
  x_test[i+1][19] = out[0][3]
  x_test[i+1][20] = out[0][4]

output = np.array(output)

print(output)

fig, axs = plt.subplots(5)
for i in range(5):
  axs[i].plot(y_test[:, i])
  axs[i].plot(output[:, i])

plt.show()
'''

fig, axs = plt.subplots(5)
for i in range(5):
  axs[i].plot(y_test[:, i])
  axs[i].plot(model.predict(x_test)[:, i])
axs[3].set_ylim([0.2, 0.8])
plt.show()


'''
models:
1:
model = tf.keras.Sequential()
model.add(layers.Dense(21, activation='sigmoid', input_shape=(21,)))
model.add(layers.Dropout(0.01))
model.add(layers.Dense(10, activation='linear'))
model.add(layers.Dropout(0.01))
model.add(layers.Dense(5))

2:
model = tf.keras.Sequential()
model.add(layers.Dense(21, activation='sigmoid', input_shape=(21,)))
model.add(layers.Dropout(0.01))
model.add(layers.Dense(10, activation='sigmoid'))
model.add(layers.Dropout(0.01))
model.add(layers.Dense(5))

3:
model = tf.keras.Sequential()
model.add(layers.Dense(21, activation='sigmoid', input_shape=(21,)))
model.add(layers.Dropout(0.01))
model.add(layers.Dense(5))

4:
model = tf.keras.Sequential()
model.add(layers.Dense(16, activation='relu', input_shape=(16,)))
model.add(layers.Dense(100, activation='relu'))
model.add(layers.Dense(100, activation='relu'))
model.add(layers.Dense(50, activation='relu'))
model.add(layers.Dense(25, activation='relu'))
model.add(layers.Dense(10, activation='relu'))
model.add(layers.Dense(5))

5:

model = tf.keras.Sequential()
model.add(layers.Dense(16, activation='relu', input_shape=(16,)))
model.add(layers.Dense(20, activation='relu'))
model.add(layers.Dense(20, activation='relu'))
model.add(layers.Dense(20, activation='relu'))
model.add(layers.Dense(20, activation='relu'))
model.add(layers.Dense(20, activation='relu'))
model.add(layers.Dense(20, activation='relu'))
model.add(layers.Dense(10, activation='relu'))
model.add(layers.Dense(5))

16:
model = tf.keras.Sequential()
model.add(layers.Dense(30, activation='relu', input_shape=(16,)))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(5))

17:
with recurent
model = tf.keras.Sequential()
model.add(layers.Dense(30, activation='relu', input_shape=(21,)))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(5))

18:
model = tf.keras.Sequential()
model.add(layers.Dense(30, activation='relu', input_shape=(16,)))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(30, activation='relu'))
model.add(layers.Dense(5))
'''