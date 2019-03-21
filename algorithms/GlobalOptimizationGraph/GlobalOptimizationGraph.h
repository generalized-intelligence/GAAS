class GlobalOptimizationGraph
{
public:
    void addBlockAHRS();
    void addBlockSLAM();
    void addBlockQRCode();
    void addBlockSceneRetriever();
    void addBlockFCAttitude();



private:
    State state;
}
