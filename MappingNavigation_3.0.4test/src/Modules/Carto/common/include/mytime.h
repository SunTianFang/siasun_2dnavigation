

#ifndef COMMON_TIME_H_
#define COMMON_TIME_H_


#include <ctime>
#include <limits>
#include <ostream>
#include <ratio>

#include "port.h"



#include <sys/time.h>



using namespace std;


namespace common {



class Time{

public:

	Time(){sec = 0;usec = 0; }


	Time(const double time_){

		sec = (long int)time_;
		usec = (long int)((time_-sec)*1000000ll);

		

	}


	Time(const Time &time_)
	{
		sec = time_.sec;
		usec = time_.usec;
	
	}
	Time(long int sec_, long int usec_)
	{
		sec = sec_;
		usec = usec_;

	}

	
	bool operator<(const Time &time_) const
	{
		if(sec<time_.sec)
			return true;
		else if (sec == time_.sec)
		{
			if(usec <time_.usec)
				return true;
			else
				return false;
		}
		else
			return false;
		
	}
	bool operator>(const Time &time_) const
	{
		if(sec>time_.sec)
			return true;
		else if (sec == time_.sec)
		{
			if(usec >time_.usec)
				return true;
			else
				return false;
		}
		else
			return false;
		
	}
	bool operator<=(const Time &time_) const 
	{
		if(sec<time_.sec)
			return true;
		else if (sec == time_.sec)
		{
			if(usec <=time_.usec)
				return true;
			else
				return false;
		}
		else
			return false;
		
	}


	bool operator>=(const Time &time_) const 
	{
		if(sec>time_.sec)
			return true;
		else if (sec == time_.sec)
		{
			if(usec >=time_.usec)
				return true;
			else
				return false;
		}
		else
			return false;
		
	}
	bool operator==(const Time &time_) const
	{
		if(sec==time_.sec && usec ==time_.usec)
			return true;
		else
			return false;
	}

	bool operator!=(const Time &time_) const
	{
		if(sec!=time_.sec || usec !=time_.usec)
			return true;
		else
			return false;
	}
	Time& operator=(const Time &time_) 
	{
		sec = time_.sec;
		usec = time_.usec;
		return *this;
	}

/*
	Time operator-(const Time& rhs)  const
    { 
		long res_sec;
		long res_usec;

		if(usec>=rhs.usec)
		{
			res_usec = usec - rhs.usec;
			res_sec = sec - rhs.sec;

		}
		else
		{
			res_sec = sec-rhs.sec-1;
			res_usec = 1000000+usec-rhs.sec;
		}
			
		Time time(res_sec,res_usec );
		return time; 

	}*/


	double operator-(const Time& rhs)  const
    { 
		double lsec = ((double)usec)/1000000.0 + (double)sec;
		double rsec = ((double)rhs.usec)/1000000.0 + (double)rhs.sec;
		return (lsec-rsec); 

	}


	Time operator-(const double rhs)  const
    { 

		double tmp = ((double)usec)/1000000.0 + (double)sec - rhs;

		long int res_sec;
		long int res_usec;

		res_sec = (long int)tmp;
		res_usec = (long int)((tmp-res_sec)*1000000ll);
			
		Time time(res_sec,res_usec );
		return time; 

	}
	Time operator+(const double rhs)  const
    { 

		double tmp = ((double)usec)/1000000.0 + (double)sec + rhs;

		long int res_sec;
		long int res_usec;

		res_sec = (long int)tmp;
		res_usec = (long int)((tmp-res_sec)*1000000ll);
			
		Time time(res_sec,res_usec );
		return time; 

	}



	//friend Time operator-(const Time& lhs, const Time& rhs);



	Time operator+(const Time& rhs) const
    {
		long int res_sec = sec + rhs.sec;
		long int res_usec = usec + rhs.usec;

		if(res_usec>=1000000)
		{
			res_sec+= 1;
			res_usec-= 1000000;
		}			
			
		Time time(res_sec, res_usec);
		return time; 

     }

	 Time& operator+=(const Time& rhs)
     { 
		sec += rhs.sec;
		usec += rhs.usec;

		if(usec>=1000000)
		{
			sec+= 1;
			usec-= 1000000;
		}		

		return *this; 
      }

	Time& operator-=(const Time& rhs) 
    { 

		if(usec>=rhs.usec)
		{
			usec = usec - rhs.usec;
			sec = sec - rhs.sec;

		}
		else
		{
			sec = sec-rhs.sec-1;
			usec = 1000000+usec-rhs.sec;
		}

		return *this; 
	}


	long int Sec() const {return sec;}
	long int Usec() const {return usec;}


	double ToSecond() const
	{
		double res = ((double)usec)/1000000.0 + (double)sec;

		return res;
		

	}
	static Time min()
	{
		Time time(numeric_limits<long int>::lowest());

		return time;

	}
	static Time max()
	{
		Time time(numeric_limits<long int>::max());

		return time;

	}

	static Time now()
	{

		Time time_;
	 	struct  timeval    tv;

		gettimeofday(&tv,NULL);
		time_.sec = tv.tv_sec;
		time_.usec = tv.tv_usec;
		return time_;
		
	}
	void SetTime(long int _sec, long int _usec)
	{
		sec = _sec;
		usec = _usec;
	}

        bool SaveTime(FILE *fp, Time &time)
        {
            if (fwrite(&time.sec, sizeof(long int), 1, fp) != 1)
                return false;

            if (fwrite(&time.usec, sizeof(long int), 1, fp) != 1)
                return false;

            return true;
        }

        bool LoadTime(FILE *fp, Time &time)
        {
            long int timesec = 0;
            long int timeusec = 0;
            if (fread(&timesec, sizeof(long int), 1, fp) != 1)
                return false;

            if (fread(&timeusec, sizeof(long int), 1, fp) != 1)
                return false;

            time.SetTime(timesec,timeusec);

            return true;
        }


private:
	long int sec;
	long int usec;
	
 

};




double ToSeconds(Time time);
std::ostream& operator<<(std::ostream& os, Time time);

/*
Time operator-(const Time& lhs, const Time& rhs)
{
		long res_sec;
		long res_usec;

		if(lhs.usec>=rhs.usec)
		{
			res_usec = lhs.usec - rhs.usec;
			res_sec = lhs.sec - rhs.sec;

		}
		else
		{
			res_sec = lhs.sec-rhs.sec-1;
			res_usec = 1000000+lhs.usec-rhs.sec;
		}
			
		Time time(res_sec,res_usec );
		return time; 



}

*/
}  // namespace common


#endif  // COMMON_TIME_H_
