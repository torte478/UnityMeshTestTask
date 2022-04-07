using UnityEngine;

/// <summary>
/// Класс, связывающий логику и пользовательский интерфейс.
/// </summary>
public class Main : MonoBehaviour
{
    private MeasureConverter converter;

    /// <summary>
    /// Количество мм в 1 условной единице измерения.
    /// </summary>
    public int UnitMm;

    public Solver Solver;

    void Start()
    {
        converter = new MeasureConverter(UnitMm);

        var gui = GetComponent<GUI>();
        gui.InitInputFields(20, 30, 40);
        gui.OnInputChange += OnInputChange;

        OnInputChange(20, 30, 40);
    }

    private void OnInputChange(int offset, int angle, int shift)
    {
        var area = Solver.Run(
            converter.MmToUnits(offset),
            angle,
            converter.MmToUnits(shift)
        );

        var result = converter.SqrUnitsToSqrSm(area).ToString("0.00");

        GetComponent<GUI>().UpdateResultText(result);
    }
}
