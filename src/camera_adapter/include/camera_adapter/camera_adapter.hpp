class CameraAdapter
{

public:

virtual bool open();

virtual bool start();

virtual bool stop();


virtual void publishRGB();

virtual void publishDepth();

};