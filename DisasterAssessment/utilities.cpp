//
//  utilities.cpp
//  DisasterAssessment
//
//  Created by Allen Tang on 5/1/18.
//  Copyright © 2018 Allen Tang. All rights reserved.
//

#include "utilities.hpp"

void saveEigenMatrix(Eigen::Matrix4f &src, std::string pathname) {
    std::ofstream file(pathname);
    if (file.is_open())
    {
        file << std::setprecision(20) << src;
        file.close();
    }
}

void loadEigenMatrix(Eigen::Matrix4f &src, std::string pathname) {
//    std::ifstream file(pathname);
//    if (file.is_open()) {
//        float a;
//        int i = 0;
//        while (file >> a) {
//            src(i) = a;
//            i++;
//            if (i > src.size() - 1) {
//                break;
//            }
//        }
//        file.close();
//    }
    int cols = 0, rows = 0;
    float buff[MAXBUFSIZE];
    
    // Read numbers from file into buffer.
    std::ifstream infile;
    infile.open(pathname);
    while (! infile.eof())
    {
        std::string line;
        getline(infile, line);
        
        int temp_cols = 0;
        std::stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];
        
        if (temp_cols == 0)
            continue;
        
        if (cols == 0)
            cols = temp_cols;
        
        rows++;
    }
    
    infile.close();
    
    // Populate matrix with numbers.
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            src(i,j) = buff[ rows*i+j ];
}
