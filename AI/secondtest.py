#run with condra (virtualenv):
#conda activate tf
#tensorflow(-gpu)==2.1
#python 3.7.9

import os
os.add_dll_directory(r"C:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v10.1\bin")
os.add_dll_directory(r"D:\INSTALATORY\cudnn-10.1-windows10-x64-v8.0.4.30\cuda\bin")

import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import math
import random
from tensorflow.keras import layers

nsamples = 1000
val_ratio = 0.2
test_ratio = 0.2
tflite_model_name = "second_model"
c_model_name = "second_model"

np.random.seed(1234)

x1_values = np.random.uniform(low=0, high=(2 * math.pi), size=nsamples)
x2_values = np.random.uniform(low=0, high=(2 * math.pi), size=nsamples)

x_values = []
y_values = []

for i in range(nsamples):
  x1 = random.random() * 2 * 3.14
  x2 = random.random() * 2 * 3.14
  y1 = math.sin(x1) + (random.random() - 0.5) * 0.1
  y2 = math.cos(x2) + (random.random() - 0.5) * 0.1
  x_values.append([x1, x2])
  y_values.append([y1, y2])

val_split = int(val_ratio * nsamples)
test_split = int(val_split + (test_ratio * nsamples))
x_val, x_test, x_train = np.split(x_values, [val_split, test_split])
y_val, y_test, y_train = np.split(y_values, [val_split, test_split])

model = tf.keras.Sequential()
model.add(layers.Dense(16, activation='relu', input_shape=(2,)))
model.add(layers.Dense(16, activation='relu'))
model.add(layers.Dense(2))

model.compile(optimizer='rmsprop', loss='mae', metrics=['mae'])

history = model.fit(x_train,
                    y_train,
                    epochs=100,
                    batch_size=100,
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

#print(model.predict([0])) 
print(model.predict(np.array([3.14, 3.14]).T))
