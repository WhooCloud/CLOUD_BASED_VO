#include "robotBase.h"

RobotJson::RobotJson()
{
	json_doc.SetObject();
	setString(" {\"index_im\" : 1, \"b\" : 2}  ");

}

vector<double> RobotJson::getDocDoubleArray(const char* key)
{
	vector<double> v_d;
	if(json_doc.HasMember(key) )
	{
		if(json_doc[key].IsArray() && json_doc[key][0].IsDouble())
		{
	        for (SizeType i = 0; i < json_doc[key].Size(); i++) // rapidjson uses SizeType instead of size_t.
            	v_d.push_back(json_doc[key][i].GetDouble());
			return v_d;
		}
		else
			cerr<< RED << RED "RobotJson::getDocIntArray: "<<key<<" is not an int array" RESET<<endl;
	}
	else
		cerr<< RED "RobotJson::getDocIntArray: "<<key<<" does not exist." RESET <<endl;
	return v_d;

}

string RobotJson::getDocString(const char* key)
{
	if(json_doc.HasMember(key) )
	{
		if(json_doc[key].IsString())
		{
			return json_doc[key].GetString();
		}
		else
			cerr<< RED << RED "RobotJson::getDocString: "<<key<<" is not a string" RESET<<endl;
	}
	else
		cerr<< RED "RobotJson::getDocString: "<<key<<" does not exist." RESET <<endl;
	return string("RobotJson::getDocString: ERROR");
}

void RobotJson::setDocString(const char* key, const string value_str)
{
	if(json_doc.HasMember(key))
	{
		json_doc[key].SetString(value_str.c_str(),value_str.length());
		docToString();	
	}
	else
	{
		cerr<< RED "RobotJson::setDocString: "<<key<<" does not exist." RESET <<endl;
	}
	return;
}

void RobotJson::addDocString(const char* key, const string value_str)
{
	Value _value_str;
	if(value_str.size() + json_str.size() > ROBOTJSON_MAXSTRINGLENGTH)
	{
		cerr<< RED "RobotJson::addDocString :"<< "value_str is too large" RESET <<endl;
		return;
	}
	char json_char_array[ROBOTJSON_MAXSTRINGLENGTH];
	stringToCharArray(json_char_array, value_str);
	_value_str.SetString(json_char_array, static_cast<SizeType>(value_str.size()), json_doc.GetAllocator());
	json_doc.AddMember( StringRef(key), _value_str, json_doc.GetAllocator());
	docToString();
	return;
}

int RobotJson::getDocInt(const char* key)
{
	if(json_doc.HasMember(key) )
	{
		if(json_doc[key].IsInt())
		{
			return json_doc[key].GetInt();
		}
		else
			cerr<< RED << RED "RobotJson::getDocInt: "<<key<<" is not an int" RESET<<endl;
	}
	else
		cerr<< RED "RobotJson::getDocInt: "<<key<<" does not exist." RESET <<endl;
	return -1;
}

void RobotJson::setDocInt(const char* key, const int value_i)
{
	if(json_doc.HasMember(key))
	{
		json_doc[key] = value_i;
		docToString();	
	}
	else
	{
		cerr<< RED "RobotJson::setDocInt: "<<key<<" does not exist." RESET <<endl;
	}
	checkString();
	return;
}

double RobotJson::getDocDouble(const char* key)
{
	if(json_doc.HasMember(key) )
	{
		if(json_doc[key].IsDouble())
		{
			return json_doc[key].GetDouble();
		}
		else
			cerr<< RED << RED "RobotJson::getDocDoublt: "<<key<<" is not a double" RESET<<endl;
	}
	else
		cerr<< RED "RobotJson::getDocDoublt: "<<key<<" does not exist." RESET <<endl;
	return -1;
}

void RobotJson::setDocDouble(const char* key, const double value_d)
{
	if(json_doc.HasMember(key))
	{
		json_doc[key] = value_d;
		docToString();	
	}
	else
	{
		cerr<< RED "RobotJson::setDocDoublt: "<<key<<" does not exist." RESET <<endl;
	}
	checkString();
	return;
}

void RobotJson::docToString()
{
    StringBuffer str_buf;
	if(json_doc.IsObject())
	{
	    PrettyWriter<StringBuffer> writer(str_buf);
	    json_doc.Accept(writer);
	    json_str = str_buf.GetString();
	}
	else
		cerr<<RED "RobotJson::docToString: json_doc is not an object."<<endl;
	checkString();
    return;
}

string RobotJson::getString()
{
	return json_str;
}

void RobotJson::setString(const string _json_str)
{
	if(_json_str.length()<ROBOTJSON_MAXSTRINGLENGTH)
	{
		json_str = _json_str;
		stringToDoc();
	}
	else
		cerr<<RED "RobotJson::setString: _json_str.length>ROBOTJSON_MAXSTRINGLENGTH." RESET<<endl;
	return;
}

void RobotJson::stringToDoc()
{
	if(json_doc.IsObject())
	{
		char json_char_array[ROBOTJSON_MAXSTRINGLENGTH];
		stringToCharArray(json_char_array, json_str);
		json_doc.Parse(json_char_array);
	}
	else
		cerr<<RED "RobotJson::stringToDoc: json_doc is not an Object." RESET<<endl;
	return;
}

void RobotJson::stringToCharArray(char * c, string s)
{
    int i;
    for( i=0; i<s.length(); i++)
        c[i] = s[i];

    c[i] = '\0';
    return;
}

void RobotJson::checkString()
{
	if(json_str.length() > ROBOTJSON_MAXSTRINGLENGTH)
		cerr<<RED "RobotJson::checkString: json_str.length() > ROBOTJSON_MAXSTRINGLENGTH." RESET<<endl;
	return;
}