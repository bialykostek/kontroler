import numpy as np
import tensorflow as tf

# Load TFLite model and allocate tensors.
interpreter = tf.lite.Interpreter(model_path="second_model.tflite")
interpreter.allocate_tensors()

# Get input and output tensors.
input_details = interpreter.get_input_details()
output_details = interpreter.get_output_details()

# Test model on random input data.
input_shape = input_details[0]['shape']
input_data = np.array([[0.58, 0.51, 0.57, 0.51, 0.59, 0.48, 0.23, 0.59, 0.48, 0.22, 0.18, -0.3, 0.53, 0.18, -0.31, 0.53, 0.43, 0.43, 0.37, 0.43, 0.43, 0.33, 0.44, 0.42, 0.64, 0.49, 0.48]], dtype=np.float32)
interpreter.set_tensor(input_details[0]['index'], input_data)

interpreter.invoke()

# The function `get_tensor()` returns a copy of the tensor data.
# Use `tensor()` in order to get a pointer to the tensor.
output_data = interpreter.get_tensor(output_details[0]['index'])
print(output_data)