#pragma once

struct BufferEntry {
	float x;
	float y;
	float velocity;
	float acceleration;
	float x_velocity;
	float y_velocity;
	float x_acceleration;
	float y_acceleration;
	float heading;
	float timeStamp;
	float timeStep;
	int obj_ID;
};

struct BufferEntryRaw {
	float x;
	float y;
	float timeStamp;
	int obj_ID;
};