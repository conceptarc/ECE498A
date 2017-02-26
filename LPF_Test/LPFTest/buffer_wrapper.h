#pragma once
#include <math.h>
#include <deque>
#include <unordered_map>
#include "BufferEntry_struct.h"

using namespace std;

class Buffer_Wrapper {
public:
	Buffer_Wrapper(); // size MUST be 3 now
	~Buffer_Wrapper();
	void Update_Object(int id, float x, float y, float time);
	void Delete_Object(int id);
	void Get_Object_Properties(int id, float &x, float &y, float &dx, float &dy);

private:
	int BUFFER_SIZE;
	unordered_map<int, deque<BufferEntry>*> input_buffer_map; // unfiltered data
	unordered_map<int, deque<BufferEntry>*> output_buffer_map; // filtered data

	void InitBuffersForNewObject(BufferEntry entry);
	deque<BufferEntry> vectorCalc(deque<BufferEntry> buffer);
};

Buffer_Wrapper::Buffer_Wrapper() {
	BUFFER_SIZE = 3; // because the discrete transfer function requires this size
}

Buffer_Wrapper::~Buffer_Wrapper() {
	unordered_map<int, deque<BufferEntry>*>::iterator i;
	for (i = input_buffer_map.begin(); i != input_buffer_map.end(); i++) {
		delete i->second;
	}
	for (i = output_buffer_map.begin(); i != output_buffer_map.end(); i++) {
		delete i->second;
	}
}

void Buffer_Wrapper::InitBuffersForNewObject(BufferEntry entry)
{
	int id = entry.obj_ID;
	input_buffer_map[id] = new deque<BufferEntry>();
	output_buffer_map[id] = new deque<BufferEntry>();

	// fill up all the buffer space immediately so we don't worry about
	// cases when the buffer is missing certain indices
	for (int i = 0; i < BUFFER_SIZE; i++) {
		input_buffer_map[id]->push_back(entry);
		output_buffer_map[id]->push_back(entry);
	}
}

void Buffer_Wrapper::Update_Object(int id, float x, float y, float time) {
	BufferEntry entry; // data entry to add to the deque<>
	entry.obj_ID = id;
	entry.x = x;
	entry.y = y;
	entry.timeStamp = time;

	// add new deque<> if not existing
	if (input_buffer_map.count(id) == 0) {
		InitBuffersForNewObject(entry);
	}

	// carry on as normal (as if the obstacle is not new)
	input_buffer_map[id]->push_back(entry);
	input_buffer_map[id]->pop_front(); // enforce the max buffer size

	// generate the next output buffer element value using our discrete transfer function
	// derived from (1/(1/31.459)s+1)^2 -> discretized using trapezoidal rule at Ts = 1/30 (using Matlab)
	float uk = 0.1181f;
	float uk_1 = 0.2362f;
	float uk_2 = 0.1181f;
	float yk_1 = 0.6254f;
	float yk_2 = -0.09777f;

	BufferEntry output;
	output.x =
		uk*input_buffer_map[id]->at(2).x +
		uk_1*input_buffer_map[id]->at(1).x +
		uk_2*input_buffer_map[id]->at(0).x +
		yk_1*output_buffer_map[id]->at(1).x +
		yk_2*output_buffer_map[id]->at(0).x;

	output.y =
		uk*input_buffer_map[id]->at(2).y +
		uk_1*input_buffer_map[id]->at(1).y +
		uk_2*input_buffer_map[id]->at(0).y +
		yk_1*output_buffer_map[id]->at(1).y +
		yk_2*output_buffer_map[id]->at(0).y;

	output.timeStamp = time;

	output_buffer_map[id]->push_back(output);
	output_buffer_map[id]->pop_front();
}

void Buffer_Wrapper::Delete_Object(int id) {
	delete input_buffer_map[id];
	input_buffer_map.erase(id);
	delete output_buffer_map[id];
	output_buffer_map.erase(id);
}

void Buffer_Wrapper::Get_Object_Properties(int id, float &x, float &y, float &dx, float &dy) {
	deque<BufferEntry>* buffer = output_buffer_map[id];
	deque<BufferEntry> differentialBuffer = vectorCalc(*buffer);

	BufferEntry lastestOutput = differentialBuffer[BUFFER_SIZE - 1];

	x = lastestOutput.x;
	y = lastestOutput.y;
	dx = lastestOutput.x_velocity;
	dy = lastestOutput.y_velocity;
}

//Calculates heading (radians), velocity, acceleration for one set of coordinates for one obj
deque<BufferEntry> Buffer_Wrapper::vectorCalc(deque<BufferEntry> buffer)
{
	deque<BufferEntry> output;

	for (size_t i = 0; i < buffer.size(); i++)
	{
		BufferEntry newEntry;
		if (i > 0)
		{
			float deltaT = buffer[i].timeStamp - buffer[i - 1].timeStamp;
			newEntry.x_velocity = (buffer[i].x - buffer[i - 1].x) / deltaT;
			newEntry.y_velocity = (buffer[i].y - buffer[i - 1].y) / deltaT;
		}
		else//i == 0
		{
			newEntry.x_velocity = 0.0;
			newEntry.y_velocity = 0.0;
		}
		newEntry.x = buffer[i].x;
		newEntry.y = buffer[i].y;
		newEntry.obj_ID = buffer[i].obj_ID;
		output.push_back(newEntry);
	}
	return output;
}