/* Copyright (C) designer2k2 Stephan M.
  # This file is part of EMUcan <https://github.com/designer2k2/EMUcan>.
  #
  # EMUcan is free software: you can redistribute it and/or modify
  # it under the terms of the GNU General Public License as published by
  # the Free Software Foundation, either version 3 of the License, or
  # (at your option) any later version.
  #
  # EMUcan is distributed in the hope that it will be useful,
  # but WITHOUT ANY WARRANTY; without even the implied warranty of
  # MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  # GNU General Public License for more details.
  #
  # You should have received a copy of the GNU General Public License
  # along with EMUcan.  If not, see <http://www.gnu.org/licenses/>.

 This simulates the full life cycle of the EMUcan library.

 Based on the work from Erik Elmore:
 https://stackoverflow.com/questions/780819/how-can-i-unit-test-arduino-code
*/

#include <unistd.h>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include <string>

// Floats never compare exact, so check them against a tolerance:
static void check_float(const char *what, const float got, const float expected) {
  if (std::fabs(got - expected) > 0.001) {
    throw std::runtime_error(std::string("EMUcan ") + what + " not ok, got " + std::to_string(got));
  }
  std::cout << "EMUcan " << what << " ok: " << std::to_string(got) << std::endl;
}

#include "EMUcan.h"
#include "WProgram.h"

#if EMUCAN_USER_CHANNELS < 8
#error "the EMUcan tests map 8 user channels, build with EMUCAN_USER_CHANNELS >= 8"
#endif

void run_tests() {
  using std::cout;
  using std::endl;

  // Init the library:
  EMUcan emucan(0x600);

  // Print the library version:
  cout << "EMUcan version: " << EMUCAN_LIB_VERSION << endl;

  // Check if the init was ok, status has to be fresh:
  if (emucan.EMUcan_Status() == EMUcan_FRESH) {
    cout << "EMUcan init ok" << endl;
  } else {
    throw std::runtime_error("EMUcan init not ok");
  }

  // Generate a can frame and hand it over:
  uint8_t data[8] = { 0x00, 0x0f, 0x13, 0x02, 0x00, 0x00, 0x08, 0x00 };
  emucan.checkEMUcan(0x604, 8, data);

  // Now the status has to be that something was received:
  if (emucan.EMUcan_Status() == EMUcan_RECEIVED_WITHIN_LAST_SECOND) {
    cout << "EMUcan status update ok" << endl;
  } else {
    throw std::runtime_error("EMUcan status update not ok.");
  }

  // Based on the frame from above:
  if (emucan.emu_data.emuTemp == 15) {
    cout << "EMUcan decode 1st frame ok" << endl;
    cout << "emuTemp: " << std::to_string(emucan.emu_data.emuTemp) << endl;
  } else {
    throw std::runtime_error("EMUcan decode not ok.");
  }

  // Generate another frame:
  uint8_t data2[8] = { 0xf0, 0x02, 0x02, 0x16, 0x25, 0x00, 0x76, 0x00 };
  emucan.checkEMUcan(0x600, 8, data2);

  // Based on the frame from above:
  if (emucan.emu_data.RPM == 752) {
    cout << "EMUcan decode 2nd frame ok" << endl;
    cout << "RPM: " << std::to_string(emucan.emu_data.RPM) << endl;
  } else {
    throw std::runtime_error("EMUcan decode not ok.");
  }

  // Check the flags:
  if (emucan.emu_data.flags1 & emucan.F_IDLE) {
    cout << "EMUcan decode flags ok" << endl;
  } else {
    throw std::runtime_error("EMUcan decode flags not ok.");
  }

  // CEL should not be on:
  if (emucan.decodeCel() == false) {
    cout << "EMUcan CEL check ok" << endl;
  } else {
    throw std::runtime_error("EMUcan CEL check not ok.");
  }

  // A user defined CAN stream frame is ignored as long as nothing is mapped to it:
  uint8_t knock[8] = { 0xd3, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  if (emucan.checkEMUcan(0x60f, 8, knock) == false) {
    cout << "EMUcan unmapped user frame ignored ok" << endl;
  } else {
    throw std::runtime_error("EMUcan unmapped user frame not ignored.");
  }

  // Map the knock ignition correction of the user defined CAN stream:
  // 16 bits signed little endian at byte 0, multiplier 10 -> 0.1 deg/bit.
  float knockIgnCorrection = 123.0;
  if (emucan.addUserChannel(0x60f, 0, EMUcan::S16_LE, &knockIgnCorrection, 10) == false) {
    throw std::runtime_error("EMUcan addUserChannel not ok.");
  }
  // Registering has to give the target a defined value:
  check_float("user channel init", knockIgnCorrection, 0.0);

  // Now the very same frame has to be taken and decoded, negative as the EMU sends it:
  if (emucan.checkEMUcan(0x60f, 8, knock) == false) {
    throw std::runtime_error("EMUcan mapped user frame not taken.");
  }
  check_float("user channel signed decode", knockIgnCorrection, -4.5);

  // And a positive one:
  uint8_t knock2[8] = { 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
  emucan.checkEMUcan(0x60f, 8, knock2);
  check_float("user channel positive decode", knockIgnCorrection, 2.0);

  // A user defined frame is a message from the EMU, so it has to update the status:
  if (emucan.EMUcan_Status() == EMUcan_RECEIVED_WITHIN_LAST_SECOND) {
    cout << "EMUcan user frame status update ok" << endl;
  } else {
    throw std::runtime_error("EMUcan user frame status update not ok.");
  }

  // Arguments that could not be decoded have to be refused:
  float dummy = 0;
  if (emucan.addUserChannel(0x613, 0, EMUcan::U8, nullptr) == false
      && emucan.addUserChannel(0x613, 0, EMUcan::U8, &dummy, 0) == false
      && emucan.addUserChannel(0x613, 0, EMUcan::U8, &dummy, 1, 0) == false
      && emucan.addUserChannel(0x613, 7, EMUcan::S16_LE, &dummy) == false
      && emucan.addUserChannel(0x613, 8, EMUcan::U8, &dummy) == false) {
    cout << "EMUcan user channel argument check ok" << endl;
  } else {
    throw std::runtime_error("EMUcan user channel argument check not ok.");
  }

  // Several channels, on several message IDs, all types and scalings:
  float ain8 = 0, ain8s = 0, be = 0, bes = 0, le = 0, temperature = 0;
  if (!emucan.addUserChannel(0x610, 0, EMUcan::U8, &ain8)
      || !emucan.addUserChannel(0x610, 1, EMUcan::S8, &ain8s)
      || !emucan.addUserChannel(0x610, 2, EMUcan::U16_BE, &be)
      || !emucan.addUserChannel(0x610, 4, EMUcan::S16_BE, &bes, 1, 2)
      || !emucan.addUserChannel(0x611, 0, EMUcan::U16_LE, &le)
      || !emucan.addUserChannel(0x611, 2, EMUcan::U8, &temperature, 1, 1, -40)) {
    throw std::runtime_error("EMUcan addUserChannel of the matrix not ok.");
  }

  // One frame feeding four channels at once:
  uint8_t user1[8] = { 0xc8, 0xd3, 0x01, 0x2c, 0xff, 0xf6, 0x00, 0x00 };
  emucan.checkEMUcan(0x610, 8, user1);
  check_float("user channel U8", ain8, 200.0);
  check_float("user channel S8", ain8s, -45.0);
  check_float("user channel U16_BE", be, 300.0);
  check_float("user channel S16_BE with divider", bes, -20.0);

  // The second message ID decodes on its own:
  uint8_t user2[8] = { 0x2c, 0x01, 0x5a, 0x00, 0x00, 0x00, 0x00, 0x00 };
  emucan.checkEMUcan(0x611, 8, user2);
  check_float("user channel U16_LE", le, 300.0);
  check_float("user channel offset", temperature, 50.0);

  // A short frame must not be decoded, and must not read past its end:
  float tail = 0;
  if (emucan.addUserChannel(0x612, 6, EMUcan::S16_LE, &tail) == false) {
    throw std::runtime_error("EMUcan addUserChannel at the frame tail not ok.");
  }
  uint8_t shortframe[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff };
  emucan.checkEMUcan(0x612, 6, shortframe);
  check_float("user channel short frame", tail, 0.0);
  emucan.checkEMUcan(0x612, 8, shortframe);
  check_float("user channel full frame", tail, -1.0);

  // The table is limited and reports when it is full:
  const uint8_t filled = emucan.userChannelCount();
  for (uint8_t i = filled; i < EMUCAN_USER_CHANNELS; i++) {
    if (emucan.addUserChannel(0x614, 0, EMUcan::U8, &dummy) == false) {
      throw std::runtime_error("EMUcan user channel table not fillable.");
    }
  }
  if (emucan.addUserChannel(0x614, 0, EMUcan::U8, &dummy) == false
      && emucan.userChannelCount() == EMUCAN_USER_CHANNELS) {
    cout << "EMUcan user channel table full ok" << endl;
  } else {
    throw std::runtime_error("EMUcan user channel table full check not ok.");
  }

  // Sleep 2 seconds so that the status drops:
  sleep(2);

  // Now the status has to be that something was received:
  if (emucan.EMUcan_Status() == EMUcan_RECEIVED_NOTHING_WITHIN_LAST_SECOND) {
    cout << "EMUcan status update ok" << endl;
  } else {
    throw std::runtime_error("EMUcan status update not ok.");
  }

  cout << "EMUcan check complete, all ok." << endl;
}

int main() {
  initialize_mock_arduino();
  run_tests();
}
