#include <TensorFlowLite.h>

#include "some_model.h"
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
  
  model = tflite::GetModel(g_sine_model_data);
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
  
  float x[27] = {0.61, 0.54, 0.61, 0.53, 0.48, 0.5, 0.25, 0.48, 0.5, 0.25, 0.67, 0.26, 0.52, 0.65, 0.26, 0.52, 0.49, 0.46, -0.5, -0.47, -0.49, 0.19, 0.44, 0.6, 0.46, 0.5, 0.48};
  for(int i=0; i<1; i++){

    for(int k = 0; k < 27; k++){
   input->data.f[k] = x[k];
    }
   
   TfLiteStatus invoke_status = interpreter->Invoke();
  if (invoke_status != kTfLiteOk) {
    Serial.println("Model invoke error");
    return;
  }
  float y1 = output->data.f[0];
  float y2 = output->data.f[1];
  float y3 = output->data.f[2];
  float y4 = output->data.f[3];
  float y5 = output->data.f[4];
  Serial.print(y1);
  Serial.print(" ");
  Serial.print(y2);
  Serial.print(" ");
  Serial.print(y3);
  Serial.print(" ");
  Serial.print(y4);
  Serial.print(" ");
  Serial.println(y5);

  }
  
}

void loop() {
}
