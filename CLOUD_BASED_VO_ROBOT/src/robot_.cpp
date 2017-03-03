#include "slamBase.h"
#include "robotBase.h"

#include <sys/time.h> //getCurrentTime
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <zlib.h>

double getCurrentTime()
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (double)tv.tv_sec + (double)tv.tv_usec / 1000000.0;
}

void createPayloadCharArray(char *payload_char, const int payload_size)
{
    int i;
    for(i = 0; i < payload_size; i++)
    {
        payload_char[i] = '0';
    }
    payload_char[i] = '\0';
    return;
}

string createPayloadString(int length)
{
    string s;
    for(int i = 0; i < length; i++)
    {
        s=s+"0";
    }
    return s;
}

string matToString(cv::Mat m)
{
    stringstream ss;
    for(int i = 0; i < m.rows; i++)
        for(int j = 0; j < m.cols; j++)
            for(int d = 0; d < m.channels(); d++)
            {
                ss<<int(m.ptr<uchar>(i)[j*m.channels()+d]);
                ss<<',';
            }
    string s = ss.str();            
    return s;
}

string sizeofMatToString(cv::Mat m)
{
    stringstream ss;
    ss<<m.rows<<","<<m.cols<<","<<m.channels();
    string s;
    s = ss.str();
    return s;
}

string compressString(const string &str)
{
    char b[ROBOTJSON_MAXSTRINGLENGTH];

    z_stream defstream;
    defstream.zalloc = Z_NULL;
    defstream.zfree = Z_NULL;
    defstream.opaque = Z_NULL;
    // setup "a" as the input and "b" as the compressed output
    defstream.avail_in = (uInt)str.length()+1; // size of input, string + terminator
    defstream.next_in = (Bytef *)str.c_str(); // input char array
    defstream.avail_out = (uInt)sizeof(b); // size of output
    defstream.next_out = (Bytef *)b; // output char array
    
    // the actual compression work.
    deflateInit(&defstream, Z_BEST_COMPRESSION);
    deflate(&defstream, Z_FINISH);
    deflateEnd(&defstream);
     

    return string(b);
}

string decompressString(const string &str)
{
    char c[ROBOTJSON_MAXSTRINGLENGTH];
    z_stream infstream;
    infstream.zalloc = Z_NULL;
    infstream.zfree = Z_NULL;
    infstream.opaque = Z_NULL;
    // // setup "b" as the input and "c" as the compressed output
    infstream.avail_in = (uInt)(str.length() + 1); // size of input
    infstream.next_in = (Bytef *)str.c_str(); // input char array
    infstream.avail_out = (uInt)sizeof(c); // size of output
    infstream.next_out = (Bytef *)c; // output char array
     
    // // the actual DE-compression work.
    inflateInit(&infstream);
    inflate(&infstream, Z_NO_FLUSH);
    inflateEnd(&infstream);

    // // make sure uncompressed is exactly equal to original.
    return string(c);

}

string points2fToString(vector<cv::Point2f> points2f)
{ 
    stringstream ss;
    for(vector<cv::Point2f>::iterator it = points2f.begin();it!=points2f.end();it++)  
    {   
        ss<<it->x<<','<<it->y<<';';
    }  
    string s = ss.str();
    return s;
}

string points3fToString(vector<cv::Point3f> points3f)
{ 
    stringstream ss;
    for(vector<cv::Point3f>::iterator it = points3f.begin();it!=points3f.end();it++)  
    {   
        ss << it->x << ',' << it->y << ',' << it->z <<';';
    }  
    string s = ss.str();
    return s;
}

Eigen::Isometry3d createTMatrix(vector<double> v_d)
{
    Eigen::Isometry3d t_matrix = Eigen::Isometry3d::Identity();;
    vector<double>::iterator it = v_d.begin();
    for(int i = 0; i < 4; i++)
        for(int j = 0; j < 4; j++)
        {
            t_matrix(i, j) = *it;
            it++;
        }
    return t_matrix;
}

Eigen::Isometry3d createTMatrix(string str_tMatrix)
{
    int currpos = 0;
    int lastpos = 0;
    int row = 0;
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    while( (currpos = str_tMatrix.find(";", lastpos)) != -1 ) 
    {
        string str_temp = str_tMatrix.substr(lastpos, currpos-lastpos+1);
        // cout<<"FFIInterface: "<<"mainloop: "<<str_temp<<endl;
        float a, b, c, d;
        sscanf(str_temp.c_str(), "%f,%f,%f,%f;", &a, &b, &c, &d);
        // cout<<a<<endl;
        // cout<<b<<endl;
        // cout<<c<<endl;
        // cout<<d<<endl;
        T(row, 0) = a;
        T(row, 1) = b;
        T(row, 2) = c;
        T(row, 3) = d;
        row++;
        lastpos = currpos +1;
        if(row == 4)
            return T;
    }
}
