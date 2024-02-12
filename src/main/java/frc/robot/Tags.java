package frc.robot;

public abstract class Tags {
    public int SOURCE0;
    public int SOURCE1;
    public int SPEAKER0;
    public int SPEAKER1;
    public int AMP;
    public int STAGE0;
    public int STAGE1;
    public int STAGE2;

    public Tags() {
        init();
    }

public abstract void init();

public boolean isInTags(int tag) {
    return tag == SOURCE0 
        || tag == SOURCE1 
        || tag == SPEAKER0 
        || tag == SPEAKER1 
        || tag == AMP 
        || tag == STAGE0 
        || tag == STAGE1 
        || tag == STAGE2;
    }
}