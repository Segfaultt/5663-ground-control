namespace c
{
    class cminus
    {
    public:
		static double constrain(double value, double min, double max);
		static double clamp(double value, int op, double trigger, double clamp);
		static double constrain(double value, double min, double max, double clamp);
    };
} 
