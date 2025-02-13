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

#ifndef __MESSAGES_H__
#define __MESSAGES_H__

#include "iostream"

/**
	 * Message ID defined for system communication
	 *
	 * @brief List of available message ID
	 *
	 */
	typedef enum {
		// Generic messages
		MESSAGE_EMPTY = 0,

		// Basic log messaging (LogMessage)
		MESSAGE_LOG,

		// Message for GPIO (GpioMessage)
		MESSAGE_SET_GPIO,
		MESSAGE_GET_GPIO_REQ,
		MESSAGE_GET_GPIO_ANS
	} MessageID;


/**
 * Base class for messaging
 *
 * @brief Base class for messaging
 *
 */

class MessageHandler;

class Message {
public:

	/**
	 * Create a new, empty message
	 */
	Message() : messageID_{MESSAGE_EMPTY} {};

	/**
	 * Destroy message
	 */
	virtual ~Message() {}

	/**
	 * Translate content of message into a string that can be displayed
	 * @return A string describing message contents
	 */
	virtual std::string toString();

	/**
	 * Allocate a new mesage and copy contents of current message
	 * @return A message, copy of current
	 */
	virtual Message* copy();

	/**
	 * Compare message ID
	 * @param id Id to compare message to
	 * @return true if id is equal to message id, false otherwise
	 */
	bool compareID(MessageID id) {
		return (messageID_ == id) ? true:false;
	}

	/**
	 * Get message ID
	 * @return Current message ID
	 */
	MessageID getID() {
		return messageID_;
	}

	/**
	 * Set message ID
	 * @param id Message ID
	 */
	bool setID(MessageID id) {
		bool status=false;

		if (checkID(id)) {
			this->messageID_ = id;
			status = true;
		}

		return status;
	}

	virtual bool isValid() { return checkID(messageID_); }

	/**
	 * Comparison operator
	 * @param msg Message to be compared
	 * @return true if message are equal, false otherwise
	 */
	virtual bool operator==(const Message& msg) {
		return (messageID_ == msg.messageID_);
	}

	/**
	 * Difference operator
	 * @param msg Message to be compared
	 * @return true if message are different, false otherwise
	 */
	virtual bool operator!=(const Message& msg) {
		return !(messageID_ == msg.messageID_);
	}

protected:
	/**
	 * Message identifier (@see MessageID)
	 */
	MessageID messageID_;

	/**
	 * Verify if message ID is compatible with current message type
	 * @param id Message ID
	 * @return true, if message ID is acceptable, false otherwise
	 */
	virtual bool checkID(MessageID id);
};


class LogMessage : public Message {
public:
	LogMessage() { messageID_ = MESSAGE_LOG; }
	LogMessage(std::string &str);
	LogMessage(char* str);

	LogMessage* copy();

	std::string getString() { return str_; }

	void setString(std::string s) {str_ = s; }
	void setString(char* s) {str_ = std::string(s); }

	/**
     * Comparison operator
     * @param msg Message to be compared
     * @return true if message are equal, false otherwise
     */
    virtual bool operator==(const LogMessage& msg) {
        return ((messageID_ == msg.messageID_) && (str_ == msg.str_));
    }

    /**
     * Difference operator
     * @param msg Message to be compared
     * @return true if message are different, false otherwise
     */
    virtual bool operator!=(const LogMessage& msg) {
        return !((messageID_ == msg.messageID_) && (str_ == msg.str_));
    }
protected:
    /**
     * Message integer value
     */
    std::string str_;

    /**
     * Verify if message ID is compatible with current message type
     * @param id Message ID
     * @return true, if message ID is acceptable, false otherwise
     */
    bool checkID(MessageID id);
};

#endif /* __MESSAGES_H__ */

