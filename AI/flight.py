#run with condra (virtualenv):
#conda activate tf
#tensorflow(-gpu)==2.1
#python 3.7.9

import os
os.add_dll_directory(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v10.1\bin")
os.add_dll_directory(r"D:\INSTALATORY\cudnn-10.1-windows10-x64-v8.0.4.30\cuda\bin")
import sys
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import math
import random
from tensorflow.keras import layers

file = open('log_27_10_14_40_41.txt', 'r')

inputdata = []
inputres = []
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
      for i in range(27):
        tmp.append(float(line[i]))
      inputdata.append(tmp)
      inputres.append([float(line[27]), float(line[28]), float(line[29]), float(line[30]), float(line[31])])
    except:
      pass

nsamples = len(inputres)
val_ratio = 0.2
test_ratio = 0.1
tflite_model_name = "second_model"
c_model_name = "second_model"

val_split = int(val_ratio * nsamples)
test_split = int(val_split + (test_ratio * nsamples))
x_val, x_test, x_train = np.split(inputdata, [val_split, test_split])
y_val, y_test, y_train = np.split(inputres, [val_split, test_split])

print(x_val)
print("--------------------------------------------------------")
print(y_val)

model = tf.keras.Sequential()
model.add(layers.Dense(27, activation='sigmoid', input_shape=(27,)))
model.add(layers.Dense(5))

model.compile(optimizer='rmsprop', loss='mean_squared_error', metrics=['mse'])

history = model.fit(x_train,
                    y_train,
                    epochs=45,
                    batch_size=1000,
                    validation_data=(x_val, y_val))

loss = history.history['loss']
val_loss = history.history['val_loss']

epochs = range(1, len(loss) + 1)

plt.plot(epochs, loss, 'bo', label='Training loss')
plt.plot(epochs, val_loss, 'b', label='Validation loss')
plt.title('Training and validation loss')
plt.legend()
plt.show()

# Convert Keras model to a tflite model
converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.optimizations = [tf.lite.Optimize.OPTIMIZE_FOR_SIZE]
tflite_model = converter.convert()

open(tflite_model_name + '.tflite', 'wb').write(tflite_model)


# Function: Convert some hex value into an array for C programming
def hex_to_c_array(hex_data, var_name):

  c_str = """#define TENSORFLOW_LITE_EXPERIMENTAL_MICRO_EXAMPLES_HELLO_WORLD_SINE_MODEL_DATA_H_
#ifdef __has_attribute
#define HAVE_ATTRIBUTE(x) __has_attribute(x)
#else
#define HAVE_ATTRIBUTE(x) 0
#endif
#if HAVE_ATTRIBUTE(aligned) || (defined(__GNUC__) && !defined(__clang__))
#define DATA_ALIGN_ATTRIBUTE __attribute__((aligned(4)))
#else
#define DATA_ALIGN_ATTRIBUTE
#endif"""

  c_str += '\nconst int g_' + var_name + '_data_len = ' + str(len(hex_data)) + ';\n'
  c_str += 'const unsigned char g_' + var_name + '_data[] DATA_ALIGN_ATTRIBUTE = {'
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

print("Training finished!")
print(y_train[0])
print(y_train[1])
print(y_train[2])
print(model.predict(x_train[:]))

plt.plot(epochs, loss, 'bo', label='Training loss')
plt.plot(epochs, val_loss, 'b', label='Validation loss')
plt.title('Training and validation loss')
plt.legend()
plt.show()


'''
forplotinp = []
forplotout = []
for i in range(100):
  print(np.array2string(model.predict([inputdata[i]])), inputres[i]) 
  forplotinp.append(inputres[i][2])
  forplotout.append(model.predict([inputdata[i]])[0][2])

plt.plot(forplotinp, 'r', forplotout, 'b')
plt.show()
print("Evaluate on test data")
results = model.evaluate(x_test, y_test, batch_size=128)
print("test loss, test acc:", results)

# Generate predictions (probabilities -- the output of the last layer)
# on new data using `predict`
print("Generate predictions for 3 samples")
predictions = model.predict(x_test[:3])
print("predictions shape:", predictions.shape)
'''