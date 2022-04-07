
using UnityEngine;

public class MeasureConverter
{
    private readonly float unitMm;

    public MeasureConverter(float unitMm)
    {
        this.unitMm = unitMm;
    }

    public float MmToUnits(int value)
    {
        return Mathf.Max(value / unitMm, Mathf.Epsilon);
    }

    public float SqrUnitsToSqrSm(float value)
    {
        return unitMm * value / 100f;
    }
}
