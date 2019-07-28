class FIR_Filter
{
public:
    FIR_Filter(std::vector<double> filter)
    {
        this->filter = filter;
        size_ = filter.size();
        this->buffer.resize(size_);
        for(int i=0;i<size_;i++)
        {
            this->buffer[i] = 0.0;
        }
    }
    double update(double input_val)
    {
        for(int i=0;i<this->size_-1;i++)
        {
            this->buffer[i] = this->buffer[i+1];
        }
        this->buffer[size_-1] = input_val;
        
        double output_val = 0;
        for(int j=0;j<this->size_;j++)
        {
            output_val+= buffer[i]*filter[i];
        }
        return output_val;
    }
private:
    std::vector<double> filter;
    std::vector<double> buffer;
    int size;
};
void decompose_quaternion()
{
    //分解四元数，第一次旋转：转轴在xOy平面上；第二次旋转：转轴为z.
    // q = q1*q2;  q = cos(theta/2),(l,m,n)sin(theta/2)
    // 其中，q1满足：n = 0;
    // q2满足：l,m = 0
}

