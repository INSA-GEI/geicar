/*
 * Copyright (C) 2025 dimercur
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "messages.h"
#include "messagehandler.h"

/*
 * @brief Constants used with ToString method, for printing message id
 */

const std::string MESSAGE_ID_STRING[] = {
		"Empty",
		"Log"
};

/**
 * Create a new, empty message
 */
Message::Message(MessageHandler &from, MessageHandler &to) : from_(&from), to_(&to) {
	messageID_ = MESSAGE_EMPTY;
}

/**
 * Create a new, empty message
 */
Message::Message(MessageHandler &from, MessageHandler &to, MessageID id) : from_(&from), to_(&to) {
	setID(id);
}

/**
 * Destroy message
 */
Message::~Message() {
	// TODO: bien verifier que l'on a supprimé les allocations de données
}

/**
 * Translate content of message into a string that can be displayed
 * @return A string describing message contents
 */
std::string Message::toString() {
	if (checkID(messageID_))
		return "Message: \"" + MESSAGE_ID_STRING[this->messageID_] + "\"";
	else
		return (char*)"Invalid message";
}

/**
 * Allocate a new mesage and copy contents of current message
 * @return A message, copy of current
 */
Message* Message::copy() {
	Message *msg = new Message(*from_, *to_, messageID_);

	return msg;
}

/**
 * Check message ID
 * @return Current message ID
 */
bool Message::checkID(MessageID id) {
	if (id == MESSAGE_EMPTY) {
		return true;
	} else return false;
}

/**
 * Send current message
 * @return true if message was correctly posted, false otherwise
 */
bool Message::send() {
	//MessageHandler* msgHandler = static_cast<MessageHandler*>(to_);

	return to_->send(this);
}

/**
 * Send current message from an ISR
 * @return true if message was correctly posted, false otherwise
 */
bool Message::sendFromISR() {
	MessageHandler* msgHandler = static_cast<MessageHandler*>(to_);

	return msgHandler->sendFromISR(this);
}



/**
 * Create a new, empty message
 */
LogMessage::LogMessage(MessageHandler &from, MessageHandler &to, std::string &str) {
	Message::setSender(from);
	Message::setReceiver(to);
	LogMessage::str_ = str;
	LogMessage::setID(MESSAGE_LOG);
}


LogMessage* LogMessage::copy() {
	LogMessage *msg = new LogMessage(*from_, *to_, str_);

	return msg;
}

std::string LogMessage::getString() {
	return str_;
}

/**
 * Check message ID
 * @return Current message ID
 */
bool LogMessage::checkID(MessageID id) {
	if (id == MESSAGE_LOG) {
		return true;
	} else return false;
}

///* MessageInt */
//
///**
// * Create a new, empty int message
// */
//MessageInt::MessageInt() {
//    value = 0.0;
//}
//
///**
// * Create a new int message, with given ID and value
// * @param id Message ID
// * @param val Message value
// * @throw std::runtime_error if message ID is incompatible with int data
// */
//MessageInt::MessageInt(MessageID id, int val) {
//    MessageInt::SetID(id);
//    value = val;
//}
//
///**
// * Set message ID
// * @param id Message ID
// * @throw std::runtime_error if message ID is incompatible with float data
// */
//void MessageInt::SetID(MessageID id) {
//    if (CheckID(id))
//        messageID = id;
//    else
//        throw std::runtime_error {
//        "Invalid message id for MessageInt"
//    };
//}
//
///**
// * Translate content of message into a string that can be displayed
// * @return A string describing message contents
// */
//string MessageInt::ToString() {
//    if (CheckID(this->messageID))
//        return "Message: \"" + MESSAGE_ID_STRING[this->messageID] + "\"\nValue: " + to_string(this->value);
//    else
//        return "Invalid message";
//}
//
///**
// * Allocate a new message and copy contents of current message
// * @return A message, copy of current
// */
//Message* MessageInt::Copy() {
//    return new MessageInt(this->messageID, this->value);
//}
//
///**
// * Verify if message ID is compatible with current message type
// * @param id Message ID
// * @return true, if message ID is acceptable, false otherwise
// */
//bool MessageInt::CheckID(MessageID id) {
//    if ((id != MESSAGE_ROBOT_TURN) &&
//            (id != MESSAGE_ROBOT_MOVE)) {
//        return false;
//    } else return true;
//}
//
///* class MessageString */
//
///**
// * Create a new, empty string message
// */
//MessageString::MessageString() {
//    s = string("");
//}
//
///**
// * Create a new string message, with given ID and string
// * @param id Message ID
// * @param s Message string
// * @throw std::runtime_error if message ID is incompatible with string data
// */
//MessageString::MessageString(MessageID id, string s) {
//    MessageString::SetID(id);
//
//    this->s = s;
//}
//
///**
// * Set message ID
// * @param id Message ID
// * @throw std::runtime_error if message ID is incompatible with string data
// */
//void MessageString::SetID(MessageID id) {
//    if (CheckID(id))
//        messageID = id;
//    else
//        throw std::runtime_error {
//        "Invalid message id for MessageString"
//    };
//}
//
///**
// * Translate content of message into a string that can be displayed
// * @return A string describing message contents
// */
//string MessageString::ToString() {
//    if (CheckID(this->messageID))
//        return "Message: \"" + MESSAGE_ID_STRING[this->messageID] + "\"\nString: \"" + this->s + "\"";
//    else
//        return "Invalid message";
//}
//
///**
// * Allocate a new message and copy contents of current message
// * @return A message, copy of current
// */
//Message* MessageString::Copy() {
//    return new MessageString(this->messageID, this->s);
//}
//
///**
// * Verify if message ID is compatible with current message type
// * @param id Message ID
// * @return true, if message ID is acceptable, false otherwise
// */
//bool MessageString::CheckID(MessageID id) {
//    if ((id != MESSAGE_LOG)) {
//        return false;
//    } else return true;
//}
//
///* class MessageImg */
//
///**
// * Create a new, empty image message
// */
//MessageImg::MessageImg() {
//    this->messageID = MESSAGE_CAM_IMAGE;
//    image = NULL;
//}
//
///**
// * Create a new image message, with given ID and image
// * @param id Message ID
// * @param image Image
// * @throw std::runtime_error if message ID is incompatible with image
// */
//MessageImg::MessageImg(MessageID id, Img *image) {
//    MessageImg::SetID(id);
//    MessageImg::SetImage(image);
//}
//
///**
// * Destroy Image message
// */
//MessageImg::~MessageImg() {
//    delete (this->image);
//}
//
///**
// * Set message ID
// * @param id Message ID
// * @throw std::runtime_error if message ID is incompatible with image
// */
//void MessageImg::SetID(MessageID id) {
//    if (CheckID(id))
//        messageID = id;
//    else
//        throw std::runtime_error {
//        "Invalid message id for MessageImg"
//    };
//}
//
///**
// * Set message image
// * @param image Reference to image object
// */
//void MessageImg::SetImage(Img* image) {
//    //this->image = image->Copy();
//    this->image = image;
//}
//
///**
// * Translate content of message into a string that can be displayed
// * @return A string describing message contents
// */
//string MessageImg::ToString() {
//    if (CheckID(this->messageID))
//        return "Message: \"" + MESSAGE_ID_STRING[this->messageID] + "\"\n" + this->image->ToString();
//    else
//        return "Invalid message";
//}
//
///**
// * Allocate a new message and copy contents of current message
// * @return A message, copy of current
// */
//Message* MessageImg::Copy() {
//    return new MessageImg(this->messageID, this->image->Copy());
//}
//
///**
// * Verify if message ID is compatible with current message type
// * @param id Message ID
// * @return true, if message ID is acceptable, false otherwise
// */
//bool MessageImg::CheckID(MessageID id) {
//    if (id != MESSAGE_CAM_IMAGE) {
//        return false;
//    } else return true;
//}
//
///* class MessageBattery */
//
///**
// * Create a new, empty battery message
// */
//MessageBattery::MessageBattery() {
//    this->messageID = MESSAGE_ROBOT_BATTERY_LEVEL;
//    this->level = BATTERY_UNKNOWN;
//}
//
///**
// * Create a new battery message, with given ID and battery level
// * @param id Message ID
// * @param level Battery level
// * @throw std::runtime_error if message ID is incompatible with battery
// */
//MessageBattery::MessageBattery(MessageID id, BatteryLevel level) {
//    MessageBattery::SetID(id);
//    MessageBattery::SetLevel(level);
//}
//
///**
// * Set message ID
// * @param id Message ID
// * @throw std::runtime_error if message ID is incompatible with battery
// */
//void MessageBattery::SetID(MessageID id) {
//    if (CheckID(id))
//        messageID = id;
//    else
//        throw std::runtime_error {
//        "Invalid message id for MessageBattery"
//    };
//}
//
///**
// * Set battery level
// * @param level Battery level
// */
//void MessageBattery::SetLevel(BatteryLevel level) {
//    if ((level < BATTERY_UNKNOWN) || (level > BATTERY_FULL)) {
//        throw std::runtime_error{
//            "Invalid battery level for MessageBattery"};
//    } else {
//        this->level = level;
//    }
//}
//
///**
// * Translate content of message into a string that can be displayed
// * @return A string describing message contents
// */
//string MessageBattery::ToString() {
//    string levelString;
//
//    switch (this->level) {
//        case BATTERY_UNKNOWN:
//            levelString="Unknown";
//            break;
//        case BATTERY_EMPTY:
//            levelString="Empty";
//            break;
//        case BATTERY_LOW:
//            levelString="Low";
//            break;
//        case BATTERY_FULL:
//            levelString="Full";
//            break;
//        default:
//            levelString="Invalid";
//    }
//
//    if (CheckID(this->messageID))
//        return "Message: \"" + MESSAGE_ID_STRING[this->messageID] + "\"\nBattery level: \"" + levelString + "\"";
//    else
//        return "Invalid message";
//}
//
///**
// * Allocate a new message and copy contents of current message
// * @return A message, copy of current
// */
//Message* MessageBattery::Copy() {
//    return new MessageBattery(this->messageID, this->level);
//}
//
///**
// * Verify if message ID is compatible with current message type
// * @param id Message ID
// * @return true, if message ID is acceptable, false otherwise
// */
//bool MessageBattery::CheckID(MessageID id) {
//    if ((id != MESSAGE_ROBOT_BATTERY_LEVEL)) {
//        return false;
//    } else return true;
//}
//
///* class MessagePosition */
//
///**
// * Create a new, empty position message
// */
//MessagePosition::MessagePosition() {
//    this->messageID = MESSAGE_CAM_POSITION;
//    this->pos.angle = 0.0;
//    this->pos.robotId = 0;
//    this->pos.center.x=0.0;
//    this->pos.center.y=0.0;
//    this->pos.direction.x=0.0;
//    this->pos.direction.y=0.0;
//}
//
///**
// * Create a new position message, with given ID and position
// * @param id Message ID
// * @param pos Message position
// * @throw std::runtime_error if message ID is incompatible with string data
// */
//MessagePosition::MessagePosition(MessageID id, Position& pos) {
//    MessagePosition::SetID(id);
//    MessagePosition::SetPosition(pos);
//}
//
///**
// * Set message ID
// * @param id Message ID
// * @throw std::runtime_error if message ID is incompatible with string data
// */
//void MessagePosition::SetID(MessageID id) {
//    if (CheckID(id))
//        messageID = id;
//    else
//        throw std::runtime_error {
//        "Invalid message id for MessagePosition"
//    };
//}
//
///**
// * Set position
// * @param pos Reference to position
// */
//void MessagePosition::SetPosition(Position& pos) {
//    this->pos.angle = pos.angle;
//    this->pos.robotId = pos.robotId;
//    this->pos.center = pos.center;
//    this->pos.direction = pos.direction;
//}
//
///**
// * Translate content of message into a string that can be displayed
// * @return A string describing message contents
// */
//string MessagePosition::ToString() {
//    if (CheckID(this->messageID))
//        return "Message: \"" + MESSAGE_ID_STRING[this->messageID] + "\"\nPosition: \"" + to_string(this->pos.center.x) + ";" + to_string(this->pos.center.y) + "\"";
//    else
//        return "Invalid message";
//}
//
///**
// * Allocate a new message and copy contents of current message
// * @return A message, copy of current
// */
//Message* MessagePosition::Copy() {
//    return new MessagePosition(this->messageID, this->pos);
//}
//
///**
// * Verify if message ID is compatible with current message type
// * @param id Message ID
// * @return true, if message ID is acceptable, false otherwise
// */
//bool MessagePosition::CheckID(MessageID id) {
//    if ((id != MESSAGE_CAM_POSITION)) {
//        return false;
//    } else return true;
//}
