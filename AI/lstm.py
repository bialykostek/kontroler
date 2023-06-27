import numpy as np

X_train = np.arange(0,100,0.5) 
y_train = np.sin(X_train)

X_test = np.arange(100,200,0.5) 
y_test = np.sin(X_test)

n_features = 1

train_series = y_train.reshape((len(y_train), n_features))
test_series  = y_test.reshape((len(y_test), n_features))

from keras.preprocessing.sequence import TimeseriesGenerator

look_back  = 20

train_generator = TimeseriesGenerator(train_series, train_series,
                                      length        = look_back, 
                                      sampling_rate = 1,
                                      stride        = 1,
                                      batch_size    = 10)

test_generator = TimeseriesGenerator(test_series, test_series,
                                      length        = look_back, 
                                      sampling_rate = 1,
                                      stride        = 1,
                                      batch_size    = 10)    
import tensorflow as tf   
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import LSTM

n_neurons  = 4
model = Sequential()
#model.add(LSTM(n_neurons, input_shape=(look_back, n_features)))
model.add(Dense(1))
model.compile(optimizer='adam', loss='mse')
print(train_generator[0])
model.fit(train_generator,epochs=100)

test_predictions  = model.predict([[0]])
print(test_predictions)

tflite_model_name = "lstm_model"
c_model_name = "lstm_model"

converter = tf.lite.TFLiteConverter.from_keras_model(model)
converter.target_spec.supported_ops = [tf.lite.OpsSet.TFLITE_BUILTINS, tf.lite.OpsSet.SELECT_TF_OPS] 
converter._experimental_lower_tensor_list_ops = False
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


import matplotlib.pyplot as plt

test_predictions  = model.predict(test_generator)

x = np.arange(110,200,0.5)
fig, ax = plt.subplots(1, 1, figsize=(15, 5))
ax.plot(X_train,y_train, lw=2, label='train data')
ax.plot(X_test,y_test, lw=3, c='y', label='test data')
ax.plot(x,test_predictions[0, :, :], lw=3, c='r',linestyle = ':', label='predictions')
ax.legend(loc="lower left")
plt.show()