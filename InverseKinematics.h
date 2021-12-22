

#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

namespace InverseKinematics {

// Quick conversion from the Braccio angle system to radians
float b2a(float b){
  return b / 180.0 * PI - HALF_PI;
}

// Quick conversion from radians to the Braccio angle system
float a2b(float a) {
  return (a + HALF_PI) * 180 / PI;
}

void setup() {
	// Setup the lengths and rotation limits for each link
	Link base, upperarm, forearm, hand;

	base.init(0, b2a(0.0), b2a(180.0));
	upperarm.init(200, b2a(15.0), b2a(165.0));
	forearm.init(200, b2a(0.0), b2a(180.0));
	hand.init(270, b2a(0.0), b2a(180.0));

	// Attach the links to the inverse kinematic model
	InverseK.attach(base, upperarm, forearm, hand);
}


/**
 * Returns true if it found a solution, otherwise false
 */
bool solve(float* coordinates, float& b0, float& b1, float& b2, float& b3) {
	// The radian results
	float a0, a1, a2, a3;

  bool result;
  // Check whether the approach angle is given or not
  if (coordinates[3] >= 0) {
    result = InverseK.solve(coordinates[0], coordinates[1], coordinates[2], a0, a1, a2, a3, b2a(coordinates[3]));
  } else {
    result = InverseK.solve(coordinates[0], coordinates[1], coordinates[2], a0, a1, a2, a3);
  }

	// Now try to solve it
	if(result) {
		// We found a solution, set it
		b0 = a2b(a0);
		b1 = a2b(a1);
		b2 = a2b(a2);
		b3 = a2b(a3);
	}

  return result;
}

}

#endif
