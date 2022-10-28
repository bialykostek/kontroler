#include <TensorFlowLite.h>

#include "second_model.h"
#include "tensorflow/lite/experimental/micro/kernels/all_ops_resolver.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"
#include "tensorflow/lite/experimental/micro/micro_interpreter.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
#include "tensorflow/lite/c/c_api_internal.h"
#include "tensorflow/lite/experimental/micro/micro_error_reporter.h"

const float kXrange = 2.f * 3.14159265359f;
extern const int kInferencesPerCycle = 1000;

namespace {
  tflite::ErrorReporter* error_reporter = nullptr;
  const tflite::Model* model = nullptr;
  tflite::MicroInterpreter* interpreter = nullptr;
  TfLiteTensor* input = nullptr;
  TfLiteTensor* output = nullptr;
  int inference_count = 0;
  constexpr int kTensorArenaSize = 2 * 1024;
  uint8_t tensor_arena[kTensorArenaSize];
}

void setup() {
  Serial.begin(38400);
  Serial.println("Hello in AI test app");
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;
  
  model = tflite::GetModel(g_second_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model version error");
    return;
  }
  
  static tflite::ops::micro::AllOpsResolver resolver;
  static tflite::MicroInterpreter static_interpreter(model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk) {
    Serial.println("Allocation tensors error");
    return;
  }

  input = interpreter->input(0);
  output = interpreter->output(0);

  inference_count = 0;
}

void loop() {

  float position = static_cast<float>(inference_count) / static_cast<float>(kInferencesPerCycle);
  float x_val = position * kXrange;

for(int i=0; i <27; i++)
  input->data.f[i] = x_val;

  TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Model invoke error");
    return;
  }
  
  float y_val1 = output->data.f[0];
  float y_val2 = output->data.f[1];
    float y_val3 = output->data.f[2];
  float y_val4 = output->data.f[3];
  float y_val5 = output->data.f[4];
  
  Serial.print(y_val1);
  Serial.print(" ");
  Serial.print(y_val2);
  Serial.print(" ");
  Serial.print(y_val3);
  Serial.print(" ");
  Serial.print(y_val4);
  Serial.print(" ");
  Serial.println(y_val5);
  
  inference_count += 1;
  if (inference_count >= kInferencesPerCycle) inference_count = 0;
  delay(50);
}
