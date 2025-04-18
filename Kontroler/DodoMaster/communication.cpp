
// void queryValue(int query) {
//     switch (query) {
//     case 0:
//         //server echo
//         COM.println();
//         COM.println("#0|1");
//         break;
//     case 1:
//         //arm info
//         COM.println();
//         if (armed) {
//         COM.println("#2|1");
//         } else {
//         COM.println("#2|0");
//         }
        
//         break;
//     case 2:
//         //send waypoints
//         COM.println();
//         for (int i = 0; i < waypointsNumber; i++) {
//         COM.print(waypoints[i][0]);
//         COM.print(" ");
//         COM.println(waypoints[i][1]);
//         }
//         break;
//     case 3:
//         COM.println();
//         COM.print("#9|");
//         COM.print(leftColumnLon);
//         COM.print(";");
//         COM.println(leftColumnLat);
//         break;
//     case 4:
//         COM.println();
//         COM.print("#10|");
//         COM.print(rightColumnLon);
//         COM.print(";");
//         COM.println(rightColumnLat);
//         break;
//     default:
//         COM.print("Query ");
//         COM.print(query);
//         COM.println(" is not defined.");
//     }
// }


// void changeValue(int target, float value) {
//     bool okMessage = true;
//     switch (target) {
//     case 0:
//         if ((int) value == 0) {
//         sendingData = false;
//         } else {
//         sendingData = true;
//         }
//         break;
//     case 1:
//         sendingWaypointsIndex = (int) value;
//         break;
//     case 2:
//         waypoints[sendingWaypointsIndex / 2][sendingWaypointsIndex % 2] = (int) value;
//         EEPROM.writeInt(sendingWaypointsIndex * 2 + 1, (int) value);
//         break;
//     case 3:
//         if ((int) value == 0) {
//         leftColumnLon = longitude;
//         leftColumnLat = latitude;
//         EEPROM.writeLong(114, leftColumnLon);
//         EEPROM.writeLong(118, leftColumnLat);
//         COM.println();
//         COM.println("Left column updated");
//         }
//         break;
//     case 4:
//         if ((int) value == 0) {
//         rightColumnLon = longitude;
//         rightColumnLat = latitude;
//         EEPROM.writeLong(122, rightColumnLon);
//         EEPROM.writeLong(126, rightColumnLat);
//         COM.println();
//         COM.println("Right column updated");
//         }
//         break;
//     case 5:
//         scaleNS = value;
//         EEPROM.writeFloat(130, scaleNS);
//         COM.println();
//         COM.println("Scale NS updated");
//         break;
//     case 6:
//         scaleEW = value;
//         EEPROM.writeFloat(134, scaleEW);
//         COM.println();
//         COM.println("Scale EW updated");
//         break;
//     case 7:
//         angle = value;
//         EEPROM.writeFloat(138, angle);
//         COM.println();
//         COM.println("Angle updated");
//         break;
//     case 8:
//         currentWaypoint = (int) value;
//         calculateLine();
//         break;
//     case 9:
//         yawOffset = yaw;
//         COM.println("Compass calibrated");
//         break;
//     case 10:
//         if ((int) value == 0) {
//         sendingModeNormal = true;
//         }
//         if ((int) value == 1) {
//         sendingModeNormal = false;
//         }
//         COM.println("Sending data mode changed");
//         break;
//     case 11:
//         myGNSS.setNavigationFrequency((int) value);
//         break;
//     case 12:
//         yawOffset = 0;
//         break;
//     default:
//         okMessage = false;
//         COM.print("Value to change ");
//         COM.print(target);
//         COM.println(" is not defined.");
//         COM.clear();
//     }
//     if (okMessage) {
//         COM.println();
//         COM.println("#8|1");
//     }
// }



// void executeOrder(int order) {
//     switch (order) {
//     case 0:
//         //arm
//         COM.println();
//         Serial.println(armed);
//         armed = true;
//         altiPressOffset = GPSaltitude / 1000;
//         COM.println(altiPressOffset);
//         EEPROM.write(0, 1);
//         COM.println("#2|1");
//         Serial.println(armed);
//         break;
//     case 1:
//         //disarm
//         COM.println();
//         armed = false;
//         EEPROM.write(0, 0);
//         COM.println("#2|0");
//         break;
//     case 2:
//         sendOne = true;
//         break;
//     case 3:
//         resetFirstStep = true;
//         break;
//     case 4:
//         if (resetFirstStep) {
//         SCB_AIRCR = 0x05FA0004;
//         }
//         break;
//     default:
//         COM.print("Order ");
//         COM.print(order);
//         COM.println(" is not defined.");
//     }
// }

// void checkForMessage() {
//     if (COM.available()) {

//         int in = COM.read();

//         if (in == 46) {
//             requestComma = true;
//         }

//         if (in - '0' >= 0 && in - '0' <= 9) {
//             if (requestStatus == 1) {
//                 requestTarget *= 10;
//                 requestTarget += in - '0';
//             }
//             if (requestStatus == 2) {
//                 if (!requestComma) {
//                     requestValue *= 10;
//                     requestValue += in - '0';
//                 } else {
//                     requestValue += (float)(in - '0') / requestCommaPlace;
//                     requestCommaPlace *= 10;
//                 }
//             }
//         }

//         if (in == 59) {
//             requestStatus++;
//             if (requestStatus == 2 && requestType == 0) {
//                 queryValue(requestTarget);
//                 requestStatus = 0;
//             }
//             if (requestStatus == 2 && requestType == 1) {
//                 executeOrder(requestTarget);
//                 requestStatus = 0;
//             }
//             if (requestStatus == 3 && requestType == 2) {
//                 changeValue(requestTarget, requestValue);
//                 requestStatus = 0;
//             }
//         }

//         if (in == 63 || in == 64 || in == 33) {
//             //default
//             requestStatus = 0;
//             requestTarget = 0;
//             requestType = 0;
//             requestValue = 0;
//             requestComma = false;
//             requestCommaPlace = 10;
//         }
//         if (in == 63) {
//             requestStatus = 1;
//             requestType = 0;
//         } else if (in == 64) {
//             requestStatus = 1;
//             requestType = 1;
//         } else if (in == 33) {
//             requestStatus = 1;
//             requestType = 2;
//         }
//     }
// }
