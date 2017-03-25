#include "slamBase.h"
#include "robotBase.h"

#include <sys/time.h> //getCurrentTime
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <zlib.h>
#include <curl/curl.h>

#define _CHUNK ROBOTJSON_MAXSTRINGLENGTH
#define _COMPRESSLEVEL 9

int CompressString(const char* in_str,size_t in_len, std::string& out_str, int level);
int DecompressString(const char* in_str,size_t in_len, std::string& out_str);

string CompressString(string in_str)
{
    string out_str;
    int flag = CompressString(in_str.c_str(), in_str.length(), out_str, _COMPRESSLEVEL);
    if(!flag)
    {
        return out_str;
    }
    else
    {
        cerr<< RED "CompressString(string in_str): "<< "Compress ERROR! ERROR Code is :"<<flag<<" " RESET <<endl;
        return string("CompressString(string in_str): Compress ERROR! ");
    }
}

string DecompressString(string in_str)
{
    string out_str;
    int flag = DecompressString(in_str.c_str(), in_str.length(), out_str);
    if(!flag)
    {
        return out_str;
    }
    else
    {
        cerr<< RED "DecompressString(string in_str): "<<"Decompress ERROR! ERROR Code is :"<<flag<<" " RESET<<endl;
        return string ("DecompressString(string in_str): Decompress ERROR! ");
    }
}
/* Compress from file source to file dest until EOF on source.
   def() returns Z_OK on success, Z_MEM_ERROR if memory could not be
   allocated for processing, Z_STREAM_ERROR if an invalid compression
   level is supplied, Z_VERSION_ERROR if the version of zlib.h and the
   version of the library linked do not match, or Z_ERRNO if there is
   an error reading or writing the files. */
int CompressString(const char* in_str,size_t in_len, std::string& out_str, int level)
{
    if(!in_str)
        return Z_DATA_ERROR;

    int ret, flush;
    unsigned have;
    z_stream strm;

    unsigned char out[_CHUNK];

    /* allocate deflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    ret = deflateInit(&strm, level);
    if (ret != Z_OK)
        return ret;

    std::shared_ptr<z_stream> sp_strm(&strm,[](z_stream* strm){
         (void)deflateEnd(strm);
    });
    const char* end = in_str+in_len;

    size_t pos_index = 0;
    size_t distance = 0;
    /* compress until end of file */
    do {
        distance = end - in_str;
        strm.avail_in = (distance>=_CHUNK)?_CHUNK:distance;
        strm.next_in = (Bytef*)in_str;

        // next pos
        in_str+= strm.avail_in;
        flush = (in_str == end) ? Z_FINISH : Z_NO_FLUSH;

        /* run deflate() on input until output buffer not full, finish
           compression if all of source has been read in */
        do {
            strm.avail_out = _CHUNK;
            strm.next_out = out;
            ret = deflate(&strm, flush);    /* no bad return value */
            if(ret == Z_STREAM_ERROR)
                break;
            have = _CHUNK - strm.avail_out;
            out_str.append((const char*)out,have);
        } while (strm.avail_out == 0);
        if(strm.avail_in != 0);     /* all input will be used */
            break;

        /* done when last data in file processed */
    } while (flush != Z_FINISH);
    if(ret != Z_STREAM_END)  /* stream will be complete */
        return Z_STREAM_ERROR;

    /* clean up and return */
    return Z_OK;
}

/* Decompress from file source to file dest until stream ends or EOF.
   inf() returns Z_OK on success, Z_MEM_ERROR if memory could not be
   allocated for processing, Z_DATA_ERROR if the deflate data is
   invalid or incomplete, Z_VERSION_ERROR if the version of zlib.h and
   the version of the library linked do not match, or Z_ERRNO if there
   is an error reading or writing the files. */
int DecompressString(const char* in_str,size_t in_len, std::string& out_str)
{
    if(!in_str)
        return Z_DATA_ERROR;

    int ret;
    unsigned have;
    z_stream strm;
    unsigned char out[_CHUNK];

    /* allocate inflate state */
    strm.zalloc = Z_NULL;
    strm.zfree = Z_NULL;
    strm.opaque = Z_NULL;
    strm.avail_in = 0;
    strm.next_in = Z_NULL;
    ret = inflateInit(&strm);
    if (ret != Z_OK)
        return ret;

    std::shared_ptr<z_stream> sp_strm(&strm,[](z_stream* strm){
         (void)inflateEnd(strm);
    });

    const char* end = in_str+in_len;

    size_t pos_index = 0;
    size_t distance = 0;

    int flush = 0;
    /* decompress until deflate stream ends or end of file */
    do {
        distance = end - in_str;
        strm.avail_in = (distance>=_CHUNK)?_CHUNK:distance;
        strm.next_in = (Bytef*)in_str;

        // next pos
        in_str+= strm.avail_in;
        flush = (in_str == end) ? Z_FINISH : Z_NO_FLUSH;

        /* run inflate() on input until output buffer not full */
        do {
            strm.avail_out = _CHUNK;
            strm.next_out = out;
            ret = inflate(&strm, Z_NO_FLUSH);
            if(ret == Z_STREAM_ERROR)  /* state not clobbered */
                break;
            switch (ret) {
            case Z_NEED_DICT:
                ret = Z_DATA_ERROR;     /* and fall through */
            case Z_DATA_ERROR:
            case Z_MEM_ERROR:
                return ret;
            }
            have = _CHUNK - strm.avail_out;
            out_str.append((const char*)out,have);
        } while (strm.avail_out == 0);

        /* done when inflate() says it's done */
    } while (flush != Z_FINISH);

    /* clean up and return */
    return ret == Z_STREAM_END ? Z_OK : Z_DATA_ERROR;
}

void uploadFileHTTP(const char* file_path, const char* url)
{
    CURL *curl;
    CURLcode res;
 
    struct curl_httppost *formpost=NULL;
    struct curl_httppost *lastptr=NULL;
    struct curl_slist *headerlist=NULL;
    static const char buf[] = "Expect:";

    curl_global_init(CURL_GLOBAL_ALL);

    /* Fill in the file upload field */ 
    curl_formadd(&formpost,
               &lastptr,
               CURLFORM_COPYNAME, "sendfile",
               CURLFORM_FILE, file_path,
               CURLFORM_END);

    curl = curl_easy_init();
    /* initialize custom header list (stating that Expect: 100-continue is not
     wanted */ 
    headerlist = curl_slist_append(headerlist, buf);
    if(curl) {
    /* what URL that receives this POST */ 
    curl_easy_setopt(curl, CURLOPT_URL, url);
    // if((argc == 2) && (!strcmp(argv[1], "noexpectheader")))
      /* only disable 100-continue header if explicitly requested */ 
      // curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headerlist);
    curl_easy_setopt(curl, CURLOPT_HTTPPOST, formpost);

    /* Perform the request, res will get the return code */ 
    res = curl_easy_perform(curl);
    /* Check for errors */ 
    if(res != CURLE_OK)
      fprintf(stderr, "curl_easy_perform() failed: %s\n",
              curl_easy_strerror(res));

    /* always cleanup */ 
    curl_easy_cleanup(curl);

    /* then cleanup the formpost chain */ 
    curl_formfree(formpost);
    /* free slist */ 
    curl_slist_free_all(headerlist);
    }
}

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
