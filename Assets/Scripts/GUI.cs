using UnityEngine;
using UnityEngine.UI;
using UnityEditor;

/// <summary>
/// Класс для работы с пользовательским интерфейсом.
/// </summary>
public class GUI : MonoBehaviour
{
    /// <summary>
    /// Поле для ввода размера шва между плитками.
    /// </summary>
    public InputField Offset;

    /// <summary>
    /// Поле для ввода угла плиток.
    /// </summary>
    public InputField Angle;

    /// <summary>
    /// Поле для ввода смещения плиток.
    /// </summary>
    public InputField Shift;

    /// <summary>
    /// Поле для вывода размера площади плиток.
    /// </summary>
    public Text Result;

    public delegate void Notify(int offset, int angle, int shift);

    public event Notify OnInputChange;

    void Start()
    {
        Offset.onEndEdit.AddListener(OnEndEdit);
        Angle.onEndEdit.AddListener(OnEndEdit);
        Shift.onEndEdit.AddListener(OnEndEdit);
    }

    public void InitInputFields(int offset, int angle, int shift)
    {
        Offset.text = offset.ToString();
        Angle.text = angle.ToString();
        Shift.text = shift.ToString();
    }

    public void UpdateResultText(string result)
    {
        Result.text = result;
    }

    private void OnEndEdit(string arg0)
    {
        var offset = Validate(Offset.text, 0, 200);
        var angle = Validate(Angle.text, 0, 360);
        var shift = Validate(Shift.text, 0, 200);

        var handler = OnInputChange;
        if (handler != null)
            OnInputChange.Invoke(offset, angle, shift);
    }

    private int Validate(string text, int min, int max)
    {
        if (!int.TryParse(text, out var result) || result < min || result > max)
        {
            EditorUtility.DisplayDialog(
                "Внимание!",
                $"Введите число в диапазоне ${min}-${max}",
                "OK");
        }

        return result;
    }
}
