using System; // Necessary for basic data types, exceptions, etc.
using System.Windows.Forms; // Necessary for Windows Forms applications

namespace telemetry_data_processor 
{
    internal static class Program
    {
        /// <summary>
        ///  The main entry point for the application.
        /// </summary>
        [STAThread]
        static void Main()
        {
            // To customize application configuration such as set high DPI settings or default font,
            // see https://aka.ms/applicationconfiguration.
            ApplicationConfiguration.Initialize(); // Initialize the application configuration
            Application.Run(new Form1()); // Run the application with the main form
        }
    }
}