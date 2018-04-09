#include"DigitRecognizer.h"

/*
preprocessRGB(Mat& image,Mat& result)
thresholding by RGB value, red > 200 && red > 1.1 * blue
colse to eliminate tiny gaps and noise
*/
void DigitRecognizer::preprocessRGB(Mat& image,Mat& result)
{
    vector<Mat> channels;
    split(image,channels); 
    Mat Red = channels.at(2);
    Mat Blue = channels.at(0);
    for(int i = 0 ; i < Red.rows; i++)                                                // TODO : try to speed up this process , NOT ROBUST ENOUGH
    {
        for(int j = 0; j < Red.cols;j++)
        {
            if(Red.at<uchar>(i,j)>190 && Red.at<uchar>(i,j)>1.05*Blue.at<uchar>(i,j))
            {
                Red.at<uchar>(i,j) = 255;
            }
            else
            {
                Red.at<uchar>(i,j) = 0;
            }
        }
    }
    threshold(Red,Red,200,255,THRESH_BINARY);
    morphologyEx(Red,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(7,7)));
    imshow("result",result);
    return ;
}

void DigitRecognizer:: preprocessHSV(Mat& image, Mat& result)
{
    Mat tempHSV;
	vector<Mat> hsvSplit;
	cvtColor(image, tempHSV, COLOR_BGR2HSV);
	split(tempHSV, hsvSplit);
	equalizeHist(hsvSplit[2], hsvSplit[2]);
	merge(hsvSplit, tempHSV);
	inRange(tempHSV, Scalar(0,35, 10), Scalar(230, 255,255), result);
    morphologyEx(result,result,MORPH_CLOSE,getStructuringElement(MORPH_RECT,Size(7,7)));
	imshow("HSV", result);
    return;
}
bool DigitRecognizer::findDigits(Mat& binary)
{
    vector<vector<Point2i>> contours;
    contours.clear();
    vector<Vec4i> hierarchy;
    findContours(binary,contours,hierarchy,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);
    vector<Rect> boundRect(contours.size());
    vector<Rect> possibleTargetRects;
    vector<vector<Point>> contours_poly(contours.size());
    for(int i = 0 ; i < contours.size();i++) 
    {
        Mat a = Mat(contours[i]);
        approxPolyDP(Mat(contours[i]),contours_poly[i],1,true);
        boundRect[i] = boundingRect(Mat(contours_poly[i]));
        
        if(boundRect[i].area() > 500 &&  boundRect[i].area() < 25000)
        {
           possibleTargetRects.push_back(boundRect[i]);
           // rectangle(binary,boundRect[i],Scalar(255,255,255));
        }
    }
    if(possibleTargetRects.size()<5)
    {
        cout<<"original possibleTargetRects.size()<5"<<endl;
        cout<< possibleTargetRects.size()<<endl;
        return false;
    }
    // find the 3rd digit place by their distance 
    float **dist_map = new float *[possibleTargetRects.size()];
    int rsize = possibleTargetRects.size();
	for (int i = 0; i < rsize; i++)
	{
		dist_map[i] = new float[rsize];
        for (int j = 0; j < rsize; j++)
		{
			dist_map[i][j] = 0;
		}
	}
    
    for(int i = 0; i<rsize;i++)
    {
        for(int j = i+1;j <rsize;j++)
        {
            double dx  = (possibleTargetRects[i].x + possibleTargetRects[i].width/2) - (possibleTargetRects[j].x + possibleTargetRects[j].width/2); 
            double dy  = (possibleTargetRects[i].y + possibleTargetRects[i].height/2) - (possibleTargetRects[j].y + possibleTargetRects[j].height/2); 
            // more punishment on far points
            if(dy>30)
            {
                dy = dy*2;
            }
             if(dx>80) 
            {
                dx = dx*2;
            }
            double td = sqrt( 3*dy*dy); // y distance weigh more as digit have almost same hieght
            dist_map[i][j] = td;
            dist_map[j][i] = td;
        }
    }
    int center_idx = 0;
	float min_dist = 100000000;
	for (int i = 0; i < rsize; ++i)
	{
		float cur_d = 0;
		for (int j = 0; j < rsize; ++j)
		{
			cur_d += dist_map[i][j];
		}
		if (cur_d < min_dist)
		{
			min_dist = cur_d;
			center_idx = i;
		}
	}
    for (int i = 0; i < rsize; i++)
	{
		delete[] dist_map[i];
	}
	delete[] dist_map;
    // check if center place is valid judged by where we find soduku
   
    int x = possibleTargetRects[center_idx].x + possibleTargetRects[center_idx].width/2;
    int y = possibleTargetRects[center_idx].y+possibleTargetRects[center_idx].height/2;
    // rectangle(binary,possibleTargetRects[center_idx],Scalar(255,0,255),4);
    /*
    if(x > right || x < left || y < low)
    {
        return false;
    }
    */
    // kick out rectangles too high or too low
    for(vector<Rect>::iterator p = possibleTargetRects .begin();p < possibleTargetRects.end();)
    {
        if(abs((p->y + p->height/2) - y)>50)
        {
            rectangle(binary,*p,Scalar(255,255,255),1);
            p = possibleTargetRects.erase(p);
            cout<<" erase(p) -1"<<endl;
        }
        else
        {
            p++;
        }
    }
     imshow("p",binary);
    if(possibleTargetRects.size()<5)
    {
        cout<<"possibleTargetRects.size()<5"<<endl;
        return false;
    }
    sort(possibleTargetRects.begin(),possibleTargetRects.end(),[](Rect& a, Rect& b){return (a.x + a.width/2) < (b.x + b.width/2);});
    if(possibleTargetRects.size()==5)
    {
        cout<<" exactly 5"<<endl;
        for(int i = 0; i<5;i++)
        {
            targets.push_back(possibleTargetRects[i]);
        }
        return true;
    }
    else
    {
       for(int i = 1 ; i<5;i++)
        {
            if(possibleTargetRects[i] == possibleTargetRects[center_idx])
            {
                center_idx = i;
                break;
            }
        }
        if(center_idx < 2 || center_idx > (possibleTargetRects.size()-3))
        {
            cout<<"enter_idx < 2 || center_idx > (possibleTargetRects.size()-3)"<<endl;
            return false;
            /*
            int templ = center_idx -1 ;
            int tempr = center_idx + 1;
            int step;
            if(templ < 0)
            {
                step = abs((possibleTargetRects[center_idx].x+possibleTargetRects[center_idx].width/2)-(possibleTargetRects[tempr].x+possibleTargetRects[tempr].width/2));
            }
            else if(tempr >= possibleTargetRects.size())
            {
                step = abs((possibleTargetRects[center_idx].x+possibleTargetRects[center_idx].width/2)-(possibleTargetRects[templ].x+possibleTargetRects[templ].width/2));
            }
            else
            {
                int stepl = abs((possibleTargetRects[center_idx].x+possibleTargetRects[center_idx].width/2)-(possibleTargetRects[templ].x+possibleTargetRects[templ].width/2));
                int stepr = abs((possibleTargetRects[center_idx].x+possibleTargetRects[center_idx].width/2)-(possibleTargetRects[tempr].x+possibleTargetRects[tempr].width/2));
                step = (stepl + stepr)/2;
            }
            */
        } 
        else
        {
        targets.push_back(possibleTargetRects[center_idx-2]);
        targets.push_back(possibleTargetRects[center_idx-1]);
        targets.push_back(possibleTargetRects[center_idx]);
        targets.push_back(possibleTargetRects[center_idx+1]);
        targets.push_back(possibleTargetRects[center_idx+2]);
        return true;
        }
    }
}

int DigitRecognizer::recognize(Mat img)
{
    double ratio = (double)img.rows/(double)img.cols;
    if(ratio<1)
    {
        return -1;
    }
    else if(ratio>2 && ratio < 10)
    {
        return 1;
    }
    Mat row1 = img.rowRange(img.rows/3,img.rows/3 +1);    
    Mat row2 = img.rowRange(img.rows*2/3,img.rows*2/3 + 1);
    Mat col1 = img.colRange(img.cols/2,img.cols/2+1);
    int row1Count = 0;
    int row2Count = 0;
    int col1Count = 0;
    int count = 0;
    int row1Points[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int row2Points[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int col1Points[10] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    for(int i = 0; i<row1.cols -1; i++)
    {
        if(abs( img.at<uchar>(img.rows/3,i)-img.at<uchar>(img.rows/3,i+1) )>200)
        {
            row1Points[row1Count] = i;
            row1Count++;
            count++;
        }
    }
    for(int i = 0; i<row2.cols -1; i++)
    {
        if(abs(img.at<uchar>(img.rows*2/3,i)-img.at<uchar>(img.rows*2/3,i+1))>200)
        {
            row2Points[row2Count] = i;
            row2Count++;
            count++;
        }
    }
    for(int i = 0; i<col1.rows -1; i++)
    {
        if(abs(img.at<uchar>(i,img.cols/2)-img.at<uchar>(i+1,img.cols/2))>200)
        {
            col1Points[col1Count] = i;
            col1Count++;
            count++;
        }
    }
    //cout<< ": "<<row1Count << " " << row2Count << " "<< col1Count <<endl;
    if(count == 6)//possibly 7
    {
        if(row1Count == 2 && col1Count == 2 && row2Count == 2)
        {
            return 7;
        }
    }
    if(count == 8)// possibly 4
    {
        if(row1Count == 4 && col1Count == 2 && row2Count == 2)
        {
            return 4;
        }
    }
    if(count == 10)// possibly 2 3 5
    {
        if(row1Count == 2 && row2Count == 2 && col1Count == 6)
        {
            if(row1Points[0] > row1.cols/2 && row2Points[1] < row2.cols/2)
            {
                return 2;
            }
            if(row1Points[0] > row1.cols/2 && row2Points[0] > row2.cols/2)
            {
                return 3;
            }
            if(row1Points[1] < row1.cols/2 && row2Points[0] > row2.cols/2)
            {
                return 5;
            }
        }
    }
    if(count == 12)//possibly 6 9
    {
        if(row1Count == 2 && row2Count == 4 && col1Count == 6)
        {
            if(row1Points[1] < row1.cols/2 && row2Points[1] < row2.cols/2 && row2Points[2] > row2.cols/2 )
            {
                return 6;
            }
        }
        if(row1Count == 4 && row2Count == 2 && col1Count == 6)
        {
            if(row1Points[1] < row1.cols/2 && row1Points[2] > row2.cols/2 && row2Points[0] > row2.cols/2 )
            {
                return 9;
            }
        }

    }
    if(count == 14)// possibly 8
    {
        if(row1Count == 4 && row2Count == 4 && col1Count == 6)
        {
            return 8;
        }
    }

    return -1;
}


bool DigitRecognizer::getAns()
{
	float data[] = {1, 0.1, 0,  0, 1, 0};
	Mat affine(2, 3, CV_32FC1, data);
    //warpAffine(digitTemplateImgs.at(i), digitTemplateImgs.at(i), affine, digitTemplateImgs.at(i).size());
    
    for (int i = 0;i < 5; i++)
	{
		// TODO:
        // wrapperspective
        Mat a ;
        morphologyEx(binary(targets[i]),a,MORPH_ERODE,getStructuringElement(MORPH_RECT,Size(3,3)));
        ans[i] = recognize(a);
        namedWindow(to_string(i), CV_WINDOW_NORMAL);  
        imshow(to_string(i),a);
        cout<<"*" <<ans[i]<<endl;
	}
    for(int i = 0; i<5;i++)
    {
        if(ans[i]==-1)
        {
            return false;
        }
    }
    return true;
}
