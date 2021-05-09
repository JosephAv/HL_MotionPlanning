#include <Eigen/Dense>
#include <boost/algorithm/string.hpp>
#include <string>
#include <fstream>
#include <vector>
#include <iostream>

const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

#define NUM_ROWS 24
#define NUM_COLS 256

std::string file_name = "matrice_pc.csv";
std::ifstream file_stream(file_name); 
std::string data_row_str; 
std::vector<std::string> data_row; 
Eigen::MatrixXd data_matrix;

//Matrice dove verrà salvata la traiettoria calcolata, 3DoF di traslazione e 3 DoF di rotazione (Roll-Pitch-Yaw) 
Eigen::MatrixXd traiettoria_totale(6, NUM_COLS);


void carica_fpc()
{
	if (!file_stream.is_open())
	{
		std::cout << "ERROR\n";
	}

	data_matrix.resize(NUM_ROWS, NUM_COLS);

	for (std::int32_t i = 0; i < NUM_ROWS; ++i)
	{
		std::getline(file_stream, data_row_str);
		boost::algorithm::split(data_row, data_row_str, boost::is_any_of(","), boost::token_compress_on);

		for (std::int32_t j = 0; j < NUM_COLS; ++j)
		{
			data_matrix(i, j) = std::stod(data_row[j]);
		}
	}
}


void calcolo_single_dof(double punto_iniziale, double punto_finale, double vel_iniziale, double vel_finale, int i, double dt)
{
	//Estrazione valori iniziali e finali delle fPC
	double fPC0_start = data_matrix(i*4,0);
	double fPC1_start = data_matrix(i * 4 + 1, 0);
	double fPC2_start = data_matrix(i * 4 + 2, 0);
	double fPC3_start = data_matrix(i * 4 + 3, 0);
	double fPC0_end = data_matrix(i * 4, NUM_COLS-1);
	double fPC1_end = data_matrix(i * 4 + 1, NUM_COLS-1);
	double fPC2_end = data_matrix(i * 4 + 2, NUM_COLS-1);
	double fPC3_end = data_matrix(i * 4 + 3, NUM_COLS-1);

	//Calcolo delle derivate prime agli estremi delle fPC
	double fPC0dot_start = (data_matrix(i * 4, 1) - data_matrix(i * 4, 0)) / dt;
	double fPC1dot_start = (data_matrix(i * 4 + 1, 1) - data_matrix(i * 4 + 1, 0)) / dt;
	double fPC2dot_start = (data_matrix(i * 4 + 2, 1) - data_matrix(i * 4 + 2, 0)) / dt;
	double fPC3dot_start = (data_matrix(i * 4 + 3, 1) - data_matrix(i * 4 + 3, 0)) / dt;
	double fPC0dot_end = (data_matrix(i * 4, NUM_COLS-1) - data_matrix(i * 4, NUM_COLS-2)) / dt;
	double fPC1dot_end = (data_matrix(i * 4 + 1, NUM_COLS - 1) - data_matrix(i * 4 + 1, NUM_COLS - 2)) / dt;
	double fPC2dot_end = (data_matrix(i * 4 + 2, NUM_COLS - 1) - data_matrix(i * 4 + 2, NUM_COLS - 2)) / dt;
	double fPC3dot_end = (data_matrix(i * 4 + 3, NUM_COLS - 1) - data_matrix(i * 4 + 3, NUM_COLS - 2)) / dt;

	//Definizione vettore termine noto
	Eigen::Vector4d b;
	b << punto_iniziale - fPC0_start, punto_finale - fPC0_end, vel_iniziale - fPC0dot_start, vel_finale - fPC0dot_end;

	//Definizione matrice del sistema lineare
	Eigen::Matrix4d A;
	Eigen::Matrix4d A_I;
	A << 1, fPC1_start, fPC2_start, fPC3_start,
		1, fPC1_end, fPC2_end, fPC3_end,
		0, fPC1dot_start, fPC2dot_start, fPC3dot_start,
		0, fPC1dot_end, fPC2dot_end, fPC3dot_end;
	A_I = A.inverse();

	//Calcolo pesi per fPC
	Eigen::Vector4d x;
    x = A_I * b;

	//Calcolo traiettoria i-esimo DoF
	Eigen::RowVectorXd traiettoria_single_dof;
	traiettoria_single_dof = x(0) * Eigen::RowVectorXd::Ones(NUM_COLS) + data_matrix.row(i*4) + x(1) * data_matrix.row(i*4 + 1) + x(2)*data_matrix.row(i*4 + 2) + x(3)*data_matrix.row(i*4 + 3);

	traiettoria_totale.block(i,0,1,NUM_COLS) =  traiettoria_single_dof;
}

int main()
{
	carica_fpc();

	//Definizione istanti iniziale e finale
	double t_iniziale = 0;
	double t_finale = 1;

	//Definizione asse dei tempi
	Eigen::VectorXd t;
    t = Eigen::VectorXd::LinSpaced(NUM_COLS, t_iniziale, t_finale);

	double punto_iniziale;
	double punto_finale;

	Eigen::VectorXd vel_iniziale(6);
	Eigen::VectorXd vel_finale(6);

	vel_iniziale << 0, 0, 0, 0, 0, 0;
	vel_finale << 0, 0, 0, 0, 0, 0;

	Eigen::VectorXd coordinate_iniziali(6);
	Eigen::VectorXd coordinate_finali(6);

	coordinate_iniziali << 1, 1, 1, 1, 1, 1;
	coordinate_finali << 4, 4, 4, 4, 4, 4;


	for (int i = 0; i < 6; i++)
	{
		calcolo_single_dof(coordinate_iniziali(i), coordinate_finali(i), vel_iniziale(i), vel_finale(i), i, t(2)-t(1));
	}
	
	//Righe di output per verificare la correttezza della traiettoria calcolata
	Eigen::VectorXd posizione_iniziale;
	Eigen::VectorXd posizione_finale;
	posizione_iniziale = traiettoria_totale.col(0);
	posizione_finale = traiettoria_totale.col(NUM_COLS - 1);

	std::cout << "posizione iniziale:" << std::endl;
	std::cout << posizione_iniziale << std::endl;
	std::cout << "posizione finale:" << std::endl;
	std::cout << posizione_finale << std::endl;

	std::ofstream myfile;
	myfile.open("traiettoria_totale.csv");
	myfile << traiettoria_totale.format(CSVFormat);
	myfile.close();

	return 0;

}