//
// Created by wayne on 10/3/21.
//

#ifndef DBSCAN_DT_CLUSTER_FILE_H
#define DBSCAN_DT_CLUSTER_FILE_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <string.h>

using namespace std;
//将找到的pcd文件的路径放到vector中


class File {
private:
    vector<string> ret;
    string path;
    static long long int sort_lli(const string& str) {
        long long int res = 0;
        for (int i = 0; i < str.size(); i++)
        {
            res = res * 10 + str[i] - '0';

        }
        return res;
    }
    static bool comparator(const string& src1, const string& src2) {

        string find_src = ".pcd";

        long long int num1 = sort_lli(src1.substr(0, src1.find(find_src)));
        long long int num2 = sort_lli(src2.substr(0, src2.find(find_src)));
        if (num1 > num2) {
            return 0;
        }
        else
        {
            return 1;
        }
    }
    //读取目录下的pcd文件
    int getAllPath(string all_path, vector<string>& ret_path){ //参数1[in]要变量的目录  参数2[out]存储文件名
        DIR* dir = opendir(all_path.c_str()); //打开目录   DIR-->类似目录句柄的东西
        if ( dir == NULL )
        {
            cout<<all_path<<" is not a all_path or not exist!"<<endl;
            return -1;
        }

        struct dirent* d_ent = NULL;       //dirent-->会存储文件的各种属性
        char fullpath[128] = {0};
        char dot[3] = ".";                //linux每个下面都有一个 .  和 ..  要把这两个都去掉
        char dotdot[6] = "..";

        while ( (d_ent = readdir(dir)) != NULL )    //一行一行的读目录下的东西,这个东西的属性放到dirent的变量中
        {
            if ( (strcmp(d_ent->d_name, dot) != 0)
                 && (strcmp(d_ent->d_name, dotdot) != 0) )   //忽略 . 和 ..
            {
                if ( d_ent->d_type == DT_DIR ) //d_type可以看到当前的东西的类型,DT_DIR代表当前都到的是目录,在usr/include/dirent.h中定义的
                {

                    string newDirectory = all_path + string("/") + string(d_ent->d_name); //d_name中存储了子目录的名字
                    if( all_path[all_path.length()-1] == '/')
                    {
                        newDirectory = all_path + string(d_ent->d_name);
                    }

                    if ( -1 == getAllPath(newDirectory, ret_path) )  //递归子目录
                    {
                        return -1;
                    }
                }
                else   //如果不是目录
                {
                    string absolutePath = all_path + string("/") + string(d_ent->d_name);  //构建绝对路径
                    if( all_path[all_path.length()-1] == '/')  //如果传入的目录最后是/--> 例如a/b/  那么后面直接链接文件名
                    {
                        absolutePath = all_path + string(d_ent->d_name); // /a/b/1.txt
                    }
                    ret_path.push_back(absolutePath);
                }
            }
        }

        closedir(dir);
        return 0;
    }
    int getPcdPath(string pcd_path, vector<string> &ret_path){
        DIR* dir = opendir(pcd_path.c_str()); //打开目录   DIR-->类似目录句柄的东西
        if ( dir == NULL )
        {
            cout<<pcd_path<<" is not a pcd_path or not exist!"<<endl;
            return -1;
        }

        struct dirent* d_ent = NULL;       //dirent-->会存储文件的各种属性
        char fullpath[128] = {0};
        char dot[3] = ".";                //linux每个下面都有一个 .  和 ..  要把这两个都去掉
        char dotdot[6] = "..";

        while ( (d_ent = readdir(dir)) != NULL )    //一行一行的读目录下的东西,这个东西的属性放到dirent的变量中
        {
            if ( (strcmp(d_ent->d_name, dot) != 0) && (strcmp(d_ent->d_name, dotdot) != 0) )   //忽略 . 和 ..
            {
                ret_path.push_back(string(d_ent->d_name));
            }
        }

        sort(ret_path.begin(), ret_path.end(), comparator);
        //重命名
        if (ret_path[0].size()==24){
            vector<string> ret1=ret_path;
            for (int i = 0; i < ret_path.size(); ++i) {
                ret_path[i][10]=ret_path[i][11];
                ret_path[i]=ret_path[i].substr(0,ret_path[i].size()-13);
                ret_path[i]=ret_path[i]+".pcd";
                //cout<<ret[i]<<endl;
            }
            if (pcd_path[pcd_path.length()-1] == '/'){
                for (int i = 0; i < ret_path.size(); ++i) {
                    if (rename((pcd_path + ret1[i]).c_str(), (pcd_path + ret_path[i]).c_str()) != 0) {
                        perror("Error renaming file");
                        exit(-1);
                    }
                }
            }else {
                for (int i = 0; i < ret_path.size(); ++i) {
                    if (rename((pcd_path + string("/") +ret1[i]).c_str(), (pcd_path + string("/") + ret_path[i]).c_str()) != 0) {
                        perror("Error renaming file");
                        exit(-1);
                    }
                }
            }

        }



        //构建绝对路径
        if( pcd_path[pcd_path.length()-1] == '/')  //如果传入的目录最后是/--> 例如a/b/  那么后面直接链接文件名
        {
            for (int i = 0; i < ret_path.size(); ++i) {
                ret_path[i] = pcd_path + ret_path[i];
            }
        }else{
            for (int i = 0; i < ret_path.size(); ++i) {
                ret_path[i] = pcd_path + string("/") + ret_path[i];
            }
        }
        closedir(dir);
        return 0;
    }
    int getDirPath(string dir_path, vector<string> &ret_path){
        DIR* dir = opendir(dir_path.c_str()); //打开目录   DIR-->类似目录句柄的东西
        if ( dir == NULL )
        {
            cout<<dir_path<<" is not a dir_path or not exist!"<<endl;
            return -1;
        }
        struct dirent* d_ent = NULL;       //dirent-->会存储文件的各种属性
        char fullpath[128] = {0};
        char dot[3] = ".";                //linux每个下面都有一个 .  和 ..  要把这两个都去掉
        char dotdot[6] = "..";

        while ( (d_ent = readdir(dir)) != NULL )    //一行一行的读目录下的东西,这个东西的属性放到dirent的变量中
        {
            if ( (strcmp(d_ent->d_name, dot) != 0) && (strcmp(d_ent->d_name, dotdot) != 0) )   //忽略 . 和 ..
            {
                if ( d_ent->d_type == DT_DIR ) //d_type可以看到当前的东西的类型,DT_DIR代表当前都到的是目录,在usr/include/dirent.h中定义的
                {

                    string newDirectory = dir_path + string("/") + string(d_ent->d_name); //d_name中存储了子目录的名字
                    if( dir_path[dir_path.length()-1] == '/')
                    {
                        newDirectory = dir_path + string(d_ent->d_name);
                    }
                    ret_path.push_back(newDirectory);

                }

            }
        }

        closedir(dir);
        return 0;


    }

public:
//    File(string a, vector<string> b) : path(a), ret(b){}
    int set(string path) {
        this->path = path;
        return 0;
    }
    //将找到的pcd文件的路径放到vector中
    vector<string> get_pcd() {
        int flag = getPcdPath(path, ret);
        if (flag == -1) {
            exit(-1);
        }
        return ret;
    }
    //将找到的文件目录的路径放到vector中
    vector<string> get_dir() {
        int flag = getDirPath(path, ret);
        if (flag == -1) {
            exit(-1);
        }
        return ret;
    }

    vector<string> get_allPcd() {
        int flag = getAllPath(path, ret);
        if (flag == -1) {
            exit(-1);
        }
        return ret;
    }


};




#endif //DBSCAN_DT_CLUSTER_FILE_H
