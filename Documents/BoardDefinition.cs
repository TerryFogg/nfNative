




public class Connectors

public class PinController
{
    public string ControllerClass;
}

[Attribute:{"hello"}]

public class nanoFrameworkConnection
{
    public string ConnectorName;
    public string ConnectorPin;
    public PinController Controller;
}

public class Pin
{
    public string Name;
    public int value;
}

public class STM32H7B3I_DK
{


    public static struct Connectors = { Name = "Arduino",{ new Pin= { "A0", 0 },
                                                         { new Pin= { "A1", 0 },
                                                         { new Pin= { "A2", 0 } }};

    public nanoFrameworkConnection CN1_PIN1_GPIO_1 = new nanoFrameworkConnection { ConnectorName = "CN1", ConnectorPin = 1, Controller = "GPIO_Controller" }
    public nanoFrameworkConnection CN2_PIN1_GPIO_6 = new nanoFrameworkConnection { ConnectorName = "CN2", ConnectorPin = 1, Controller = "GPIO_Controller" }
    public nanoFrameworkConnection CN1_PIN1_PWM_2 = new nanoFrameworkConnection { ConnectorName = "CN1", ConnectorPin = 1, Controller = "PWM_Controller" }
    public nanoFrameworkConnection CN1_PIN1_ADC_7 = new nanoFrameworkConnection { ConnectorName = "CN1", ConnectorPin = 1, Controller = "ADC_Controller" }
    //..
    //..
    //..
    public nanoFrameworkConnection CN2_PIN1_PWM_1 = new nanoFrameworkConnection { ConnectorName = "CN2", ConnectorPin = 1, Controller = "PWM_Controller" }
    public nanoFrameworkConnection CN2_PIN1_ADC_7 = new nanoFrameworkConnection { ConnectorName = "CN2", ConnectorPin = 1, Controller = "ADC_Controller" }
}
public class AProgram
{
    void main()
    {
        STM32H7B3I_DK stm = new STM32H7B3I_DK();
       stm.
    }
}