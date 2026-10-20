  #include <WiFi.h>
  #include <PubSubClient.h>
  #include <ArduinoJson.h>

  // ---------------- WiFi / MQTT ----------------
  const char* ssid = "xxx";
  const char* password = "xxx";
  const char* mqtt_server = "broker.emqx.io";

  const char* topic_agv = "agv/coords";
  const char* topic_source = "source/coords";
  const char* topic_dist = "distance/sourdes";

  WiFiClient espClient;
  PubSubClient client(espClient);

  // ---------------- Geo structs ----------------
  struct GeoPoint { double latitude; double longitude; };
  GeoPoint sourceData = {0.0,0.0};
  GeoPoint agvData    = {0.0,0.0};
  bool isSourceSet = false;
  const double d_threshold=18;

  // ---------------- Motor pins (use your wiring) ----------------
  const int p1 = 13; // left forward
  const int p2 = 12; // left backward
  const int p3 = 26; // right backward
  const int p4 = 27; // right forward

  // -------------- Navigation tuning (CALIBRATE) --------------
  const float MOVE_TEST_SECONDS = 13.0;      // how long to move forward when testing (seconds)
  const float MOVE_CONTINUE_SECONDS = 30.0;  // new: how long to move forward in continue-forward mode (seconds)
  const float TURN_ANGLE_DEG    = 0.5;     // angle of each increment
  int MS_PER_DEGREE     = 800;       // milliseconds of pivot per degree (calibrate)
  const int SETTLING_TIME_AFTER_MOVE_MS = 2000; // wait for phone publish and ESP to receive (ms)
  const int MOTOR_ON_TIME_LOOP_DELAY_MS = 20;   // internal small delay to call client.loop()

  // Thresholds
  const double GPS_CHANGE_THRESHOLD = 1.5; // meters for meaningful change in sweep
  const double INCREASE_THRESHOLD = 20;   // meters increase to trigger re-search in continue mode

  // Limits
  const int MAX_TURNS = 8; // safety: do at most 8 turns (360°) in the search phase

  // ----------------- WiFi / MQTT helpers -----------------
  void setup_wifi() {
    Serial.print("Connecting to ");
    Serial.println(ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected");
  }
  void callback(char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<256> doc;
    DeserializationError err = deserializeJson(doc, payload, length);
    if (err) {
      Serial.print("JSON Failed: "); Serial.println(err.c_str());
      return;
    }
    double lat = doc["lat"];
    double lon = doc["lon"];
    if (String(topic) == topic_source) {
      sourceData.latitude = lat; sourceData.longitude = lon; isSourceSet = true;
      Serial.println("\n[TARGET ACQUIRED]");
      Serial.print("Target: "); Serial.print(lat,6); Serial.print(", "); Serial.println(lon,6);
    } else if (String(topic) == topic_agv) {
      agvData.latitude = lat; agvData.longitude = lon;
    }
  }

  bool isDistanceChanged(double oldD, double newD, double threshold = GPS_CHANGE_THRESHOLD) {
      return fabs(oldD - newD) > threshold;
  }

  void reconnect() {
    while (!client.connected()) {
      Serial.print("Connecting to MQTT...");
      String clientId = "ESP32-" + String(random(0xffff), HEX);
      if (client.connect(clientId.c_str())) {
        Serial.println("connected");
        client.subscribe(topic_agv);
        client.subscribe(topic_source);
      } else {
        Serial.print(" failed rc=");
        Serial.print(client.state());
        Serial.println(" try again in 2s");
        delay(2000);
      }
    }
  }

  // ---------------- Haversine distance ----------------
  double calculateDistanceMeters(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371000.0;
    double dLat = (lat2 - lat1) * PI / 180.0;
    double dLon = (lon2 - lon1) * PI / 180.0;
    double a = sin(dLat/2) * sin(dLat/2) +
              cos(lat1 * PI / 180.0) * cos(lat2 * PI / 180.0) *
              sin(dLon/2) * sin(dLon/2);
    double c = 2 * atan2(sqrt(a), sqrt(1-a));
    return R * c;
  }

  // ---------------- Motor primitives ----------------
  void leftForward()  { digitalWrite(p3,HIGH);digitalWrite(p4,LOW); }
  void leftBackward() { digitalWrite(p3, LOW);  digitalWrite(p4, HIGH); }
  void leftStop()     { digitalWrite(p3, LOW);  digitalWrite(p4, LOW);  }

  void rightForward() { digitalWrite(p1, HIGH); digitalWrite(p2, LOW); }
  void rightBackward(){ digitalWrite(p1, LOW);  digitalWrite(p2, HIGH); }
  void rightStop()    { digitalWrite(p1, LOW);  digitalWrite(p2, LOW);  }

  void stopAllMotors() { leftStop(); rightStop(); }

  // timed forward/back/pivot with client.loop inside (keeps MQTT alive)
  void moveForwardSeconds(float seconds) {
    unsigned long start = millis();
    leftForward(); rightForward();
    while (millis() - start < (unsigned long)(seconds * 1000.0)) {
      client.loop(); delay(MOTOR_ON_TIME_LOOP_DELAY_MS);
    }
    stopAllMotors();
  }
  void moveBackwardSeconds(float seconds) {
    unsigned long start = millis();
    leftBackward(); rightBackward();
    while (millis() - start < (unsigned long)(seconds * 1000.0)) {
      client.loop(); delay(MOTOR_ON_TIME_LOOP_DELAY_MS);
    }
    stopAllMotors();
  }
  void rotateLeftDegrees(float angle_deg) {
    long ms = (long)(angle_deg * MS_PER_DEGREE);
    unsigned long start = millis();
    leftBackward(); rightForward(); // pivot left (CCW)
    while (millis() - start < (unsigned long)ms) {
      client.loop(); delay(MOTOR_ON_TIME_LOOP_DELAY_MS);
    }
    stopAllMotors();
  }
  void rotateRightDegrees(float angle_deg) {
    long ms = (long)(angle_deg * MS_PER_DEGREE);
    unsigned long start = millis();
    leftForward(); rightBackward(); // pivot right (CW)
    while (millis() - start < (unsigned long)ms) {
      client.loop(); delay(MOTOR_ON_TIME_LOOP_DELAY_MS);
    }
    stopAllMotors();
  }
  void settlingDelayMs(long ms) {
    unsigned long s = millis();
    while (millis() - s < (unsigned long)ms) {
      client.loop(); delay(50);
    }
  }

  // ---------------- Persistent direction memory ----------------
  // direction_found: true after first successful sweep; means "keep moving forward in the chosen heading"
  bool direction_found = false;
  int best_turns_saved = 0;      // how many 45° steps from original (for debug)
  int best_dir_saved = 0;        // +1 = CCW-based heading, -1 = CW-based heading (for debug)
  double last_distance = 1e9;    // last distance measured in continue-forward mode

  void setup() {
    Serial.begin(9600);
    pinMode(p1, OUTPUT); pinMode(p2, OUTPUT); pinMode(p3, OUTPUT); pinMode(p4, OUTPUT);
    stopAllMotors();
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);
  }

  // ---------------- Main algorithm with persistent direction and continue-forward mode ----------------
  void loop() {
    // keep MQTT alive / reconnect if needed
    if (!client.connected()) reconnect();
    client.loop();

    static unsigned long lastProcess = 0;
    if (!isSourceSet) {
      static unsigned long lastLog = 0;
      if (millis() - lastLog > 2000) {
        lastLog = millis();
        Serial.println("Waiting for Source Coordinates...");
      }
      return;
    }

    // throttle heavy processing to ~1 Hz
    if (millis() - lastProcess < 1000) return;
    lastProcess = millis();

    // compute current distance using most recent agvData (updated via callback)
    double D_current = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
    Serial.print("[AGV] pos: "); Serial.print(agvData.latitude,6); Serial.print(", "); Serial.print(agvData.longitude,6);
    Serial.print(" | Dist: "); Serial.println(D_current);

    // stop condition
    if (D_current < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }

    // ---------- CONTINUE-FORWARD MODE ----------
    // If a best direction was already found in a prior iteration, keep moving forward along it.
    if (direction_found) {
      Serial.println("CONTINUE-FORWARD MODE: moving forward in saved best direction...");
      moveForwardSeconds(MOVE_CONTINUE_SECONDS);
      settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);

      double D_now = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
      char buf_now[64]; dtostrf(D_now,0,2,buf_now); client.publish(topic_dist, buf_now);
      Serial.print("Distance after continue-forward move: "); Serial.println(D_now);
      if (D_now < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }

      // If distance increased by more than INCREASE_THRESHOLD -> our heading is now wrong; redo sweep
      if (D_now > last_distance + INCREASE_THRESHOLD) {
        Serial.println("Distance increased in continue-forward mode -> clearing saved direction to redo sweep next iter.");
        direction_found = false;
        // do not immediately call sweep here; allow next loop iteration to enter the sweep flow
        last_distance = D_now;
        return;
      } else {
        // distance decreased or stayed same -> continue in same heading in next iteration
        last_distance = D_now;
        return;
      }
    }

    // ---------- NO SAVED DIRECTION: perform full search (baseline + first CCW test + CCW or CW sweep) ----------
    Serial.println("SEARCH MODE: performing baseline forward + tests...");

    // Baseline forward step
    Serial.println("Step: move forward (baseline test)");
    moveForwardSeconds(MOVE_TEST_SECONDS);
    settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);

    double D_after = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
    if (D_after < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }
    double rate = D_current - D_after; // positive => got closer
    Serial.print("Baseline rate (meters decreased): "); Serial.println(rate);

    // First CCW 45° test (mandatory)
    Serial.println("Begin first CCW 45° test after baseline move...");
    rotateLeftDegrees(TURN_ANGLE_DEG);
    settlingDelayMs(200);
    moveForwardSeconds(MOVE_TEST_SECONDS);
    settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);

    double D_first = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
    if (D_first < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }
    Serial.print("D_first|");
    Serial.println(D_first);

    // If GPS hasn't meaningfully changed, keep moving forward in this test until it does
    while (!isDistanceChanged(D_after, D_first)) {
      Serial.println("[GPS NO CHANGE] in first CCW test: continuing forward until distance changes...");
      moveForwardSeconds(MOVE_TEST_SECONDS);
      settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);
      D_first = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
    }

    double r_first = D_after - D_first;
    Serial.print("First CCW test rate: "); Serial.println(r_first);

    // If first CCW improved (r_first > 0) -> CCW sweep
    if (r_first > 0) {
      Serial.println("First CCW decreased distance. Entering CCW sweep to try improve further.");
      double best_rate = r_first;
      int best_turns = 1;
      bool found_decreasing = true;
      double prev_distance_for_search = D_first;

      for (int turns = 2; turns <= MAX_TURNS; ++turns) {
        Serial.print("CCW: Rotate left by "); Serial.print(TURN_ANGLE_DEG); Serial.print(" deg (turn #"); Serial.print(turns); Serial.println(")");
        rotateLeftDegrees(TURN_ANGLE_DEG);
        settlingDelayMs(200);

        Serial.print("CCW: Move forward after left turn #"); Serial.println(turns);
        moveForwardSeconds(MOVE_TEST_SECONDS);
        settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);

        double D_now = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
        if (D_now < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    } 
        Serial.print("D_now|");
        Serial.println(D_now);
        while (!isDistanceChanged(prev_distance_for_search, D_now)) {
          Serial.println("[GPS NO CHANGE] CCW sweep: continue forward until distance changes...");
          moveForwardSeconds(MOVE_TEST_SECONDS);
          settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);
          D_now = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
        }
        double r = prev_distance_for_search - D_now;
        Serial.print("CCW Turn #"); Serial.print(turns); Serial.print(" rate: "); Serial.println(r);

        if (r > 0) {
          if (r > best_rate) {
            best_rate = r;
            best_turns = turns;
            Serial.print("CCW: New best at turns="); Serial.print(best_turns); Serial.print(" rate="); Serial.println(best_rate);
          }
          prev_distance_for_search = D_now;
          continue;
        } else {
          int steps_to_revert = turns - best_turns;
          Serial.print("CCW: distance increased at turn "); Serial.print(turns);
          Serial.print(". Reverting CW by "); Serial.print(steps_to_revert * TURN_ANGLE_DEG); Serial.println(" deg to best heading.");
          for (int s = 0; s < steps_to_revert; ++s) {
            rotateRightDegrees(TURN_ANGLE_DEG);
            settlingDelayMs(200);
          }
          Serial.println("CCW: Move forward on best heading.");
          moveForwardSeconds(MOVE_TEST_SECONDS);
          settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);
          double D_after_best = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
          Serial.print("D_after_best, ");
          Serial.println(D_after_best);
          char buf3[64]; dtostrf(D_after_best,0,2,buf3); client.publish(topic_dist, buf3);
          Serial.print("Distance after switching to best CCW heading: "); Serial.println(D_after_best);
          if (D_after_best < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }

          // SAVE chosen direction for subsequent iterations
          direction_found = true;
          best_turns_saved = best_turns;
          best_dir_saved = +1; // CCW-derived heading
          last_distance = D_after_best;
          Serial.println("Direction saved (CCW). Continue-forward mode will apply next iterations.");
          return;
        }
      } // end CCW loop

      // CCW full circle done (found_decreasing true)
      Serial.println("CCW sweep completed (full circle) with improvement found earlier. Reverting to best and moving.");
      int steps_to_revert = MAX_TURNS - best_turns;
      for (int s = 0; s < steps_to_revert; ++s) {
        rotateRightDegrees(TURN_ANGLE_DEG);
        settlingDelayMs(200);
      }
      moveForwardSeconds(MOVE_TEST_SECONDS);
      settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);

      // Save direction info (best found)
      direction_found = true;
      best_turns_saved = best_turns;
      best_dir_saved = +1;
      last_distance = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
      Serial.println("Direction saved (CCW, full sweep finish).");
      return;
    }

    // If first CCW did NOT improve: revert to original and start CW sweep
    Serial.println("First CCW increased (or no change). Reverting to original heading and starting CW search.");
    rotateRightDegrees(TURN_ANGLE_DEG); // back to original
    settlingDelayMs(200);

    // Now perform CW sweep
    {
      double best_rate = -1e9;
      int best_turns = -1;
      bool found_decreasing = false;
      double prev_distance_for_search = D_after; // baseline still D_after (position after initial forward)

      for (int turns = 1; turns <= MAX_TURNS; ++turns) {
        Serial.print("CW: Rotate right by "); Serial.print(TURN_ANGLE_DEG); Serial.print(" deg (turn #"); Serial.print(turns); Serial.println(")");
        rotateRightDegrees(TURN_ANGLE_DEG);
        settlingDelayMs(200);

        Serial.print("CW: Move forward after right turn #"); Serial.println(turns);
        moveForwardSeconds(MOVE_TEST_SECONDS);
        settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);

        double D_now = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
        if (D_now < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }
        while (!isDistanceChanged(prev_distance_for_search, D_now)) {
          Serial.println("[GPS NO CHANGE] CW sweep: continue forward until distance changes...");
          moveForwardSeconds(MOVE_TEST_SECONDS);
          settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);
          D_now = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
          Serial.print("D_now|");
          Serial.println(D_now);
        }
        double r = prev_distance_for_search - D_now;
        Serial.print("CW Turn #"); Serial.print(turns); Serial.print(" rate: "); Serial.println(r);

        if (r > 0) {
          if (!found_decreasing || r > best_rate) {
            best_rate = r;
            best_turns = turns;
            found_decreasing = true;
            Serial.print("CW: New best at turns="); Serial.print(best_turns); Serial.print(" rate="); Serial.println(best_rate);
          }
          prev_distance_for_search = D_now;
          continue;
        } else {
          if (found_decreasing) {
            int steps_to_revert = turns - best_turns;
            Serial.print("CW: distance increased at turn "); Serial.print(turns);
            Serial.print(". Reverting left by "); Serial.print(steps_to_revert * TURN_ANGLE_DEG); Serial.println(" deg to best CW heading.");
            for (int s = 0; s < steps_to_revert; ++s) {
              rotateLeftDegrees(TURN_ANGLE_DEG);
              settlingDelayMs(200);
            }
            Serial.println("CW: Move forward on best heading.");
            moveForwardSeconds(MOVE_TEST_SECONDS);
            settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);
            double D_after_best = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
            char buf4[64]; dtostrf(D_after_best,0,2,buf4); client.publish(topic_dist, buf4);
            Serial.print("Distance after switching to best CW heading: "); Serial.println(D_after_best);
            if (D_after_best < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }

            // SAVE chosen direction for subsequent iterations
            direction_found = true;
            best_turns_saved = best_turns;
            best_dir_saved = -1; // CW-derived heading
            last_distance = D_after_best;
            Serial.println("Direction saved (CW). Continue-forward mode will apply next iterations.");
            return;
          } else {
            Serial.print("CW: No decrease at turn "); Serial.print(turns); Serial.println("; continuing CW sweep.");
            prev_distance_for_search = D_now;
            continue;
          }
        }
      } // end CW loop

      if (!found_decreasing) {
        Serial.println("Completed full-circle CW search and found no direction that decreases distance. Stopping for safety.");
        stopAllMotors();
        return;
      } else {
        int steps_to_revert = MAX_TURNS - best_turns;
        Serial.println("CW sweep finished; reverting to best and moving.");
        for (int s = 0; s < steps_to_revert; ++s) {
          rotateLeftDegrees(TURN_ANGLE_DEG);
          settlingDelayMs(200);
        }
        moveForwardSeconds(MOVE_TEST_SECONDS);
        settlingDelayMs(SETTLING_TIME_AFTER_MOVE_MS);

        // Save chosen direction
        direction_found = true;
        best_turns_saved = best_turns;
        best_dir_saved = -1;
        last_distance = calculateDistanceMeters(agvData.latitude, agvData.longitude, sourceData.latitude, sourceData.longitude);
        if (last_distance < d_threshold) {
      Serial.println(">>> AGV reached destination <<<");
      stopAllMotors();
      return;
    }
        Serial.println("Direction saved (CW, full sweep finish).");
        return;
      }
    } // end CW block

  } // end loop
