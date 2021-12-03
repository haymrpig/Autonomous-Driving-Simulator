#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <algorithm>
#include <climits>
#include <unordered_map>
using namespace std;

struct lane1 {
	string ID;
	string fromNodeId;
	string toNodeId;
	string rLinkId;
	string lLinkId;
	int num_waypoint;
	int laneNum;
	
	int leftLaneIndex = -1;
	int rightLaneIndex = -1;

	vector<int> linked_node_index;
	vector<vector<double>> coordinate;

	double maxX = 0;
	double minX = 5000000;
	double maxY = 0;
	double minY = 5000000;
};

struct map {
	pair<int,int> array;
	vector<int> struct_index;
};

int search(double target, vector<int>& v) {
	int start = 0, end = v.size() - 1;
	int answer;
	while (start <= end) {
		int mid = (start + end) / 2;
		if (v[mid] == target) return mid;
		else if (v[mid] < target) {
			start = mid + 1;
		}
		else {
			end = mid - 1;
			answer = end;
		}
	}
	return answer;
}
string getCoor(int* pos, string* start, string* end, string* str)
{
	string output = "";
	int startPos, endPos;
	int temp = *pos;

	startPos = str->find(*start, *pos);
	endPos = str->find(*end, startPos + 1);

	//this is for the case when R or L link is null
	if (startPos > *pos + 5)
	{
		*start = " ";
		*end = ",";
		startPos = str->find(*start, temp);
		endPos = str->find(*end, startPos + 1);
	}

	for (int i = startPos + 1; i < endPos; i++)
	{
		if ((*str)[i] == 46 || 48 <= (*str)[i] && (*str)[i] <= 57 || 65 <= (*str)[i] &&
			(*str)[i] <= 90 || 97 <= (*str)[i] && (*str)[i] <= 122)
		{
			output += ((*str)[i]);

		}
	}

	*pos = endPos;
	return output;
}

string searchNget(int* pos, string* start, string* end, string* str, string* word){
	string output;
	int nextPos = 0;

	nextPos = str->find(*word, *pos) + word->length();
	if (nextPos >= word->length())
	{
		output = getCoor(&nextPos, start, end, str);
		return output;
	}

}


int main()
{

	//  weight, from , to
	vector<pair<int,pair<int, int>>> weight;
	lane1* lane_node = new lane1[10000];

	string filename, writefile, writefile_final, startNodeID;
	string wordFind, wordFindNext = "\"coordinates\": [ [ ", wordFindNext1 = "[";
	string num_x = "", num_y = "";
	string fromnode, tonode;
	double x, y;
	vector<vector<double>> array_temp, array;

	filename = "combine.geojson";
	writefile = "nodeAndCoor.txt";
	writefile_final = "Final.txt";
	startNodeID = "A1181R780975";


	string str_temp;
	ifstream fin(filename);
	//ofstream fout(writefile);

	if (fin.is_open()) {
		cout << "file is opened" << endl;
		cout << "wait...." << endl;
	}
	else
		cout << "file not found" << endl;

	int cnt = 0;
	string start, end, word, word1;

	if (fin.is_open())
	{
		while (!fin.eof())
		{
			double maxX = 0;
			double minX = 5000000;
			double maxY = 0;
			double minY = 5000000;

			string temp;
			getline(fin, temp);
			int pos = 0;

			
			start = "\"";
			end = "\"";
			word = "\"ID\"";

			if (temp.find(word, pos) != string::npos)
			{
				//find ID
				//fout << "ID ";
				lane_node[cnt].ID = searchNget(&pos, &start, &end, &temp, &word);//,fout);
				//fout << "\n";
				
				//find Lane number
				start = " ";
				end = ",";
				word = "\"LaneNo\"";

				//fout << "LaneNo ";
				lane_node[cnt].laneNum = stod(searchNget(&pos, &start, &end, &temp, &word));//, fout));
				//fout << "\n";

				//find l,r link ID
				//fout << "rlink, llink ";
				start = "\"";
				end = "\"";
				word = "\"R_LinkID\"";
				word1 = "\"L_LinkID\"";
				lane_node[cnt].rLinkId = searchNget(&pos, &start, &end, &temp, &word);//,fout);
				lane_node[cnt].lLinkId = searchNget(&pos, &start, &end, &temp, &word1);//,fout);
				//fout << "\n";

				//find from, to node
				start = "\"";
				end = "\"";
				word = "\"FromNodeID\"";
				word1 = "\"ToNodeID\"";
				lane_node[cnt].fromNodeId = searchNget(&pos, &start, &end, &temp, &word);//,fout);
				lane_node[cnt].toNodeId = searchNget(&pos, &start, &end, &temp, &word1);//,fout);
				//fout << "\n";
				int num_waypoint = 0;
				if (temp.find(wordFindNext, pos) != string::npos)
				{
					pos = temp.find(wordFindNext, pos) + wordFindNext.length();

					while (temp.find(wordFindNext1, pos) != string::npos)
					{
						pos = temp.find(wordFindNext1, pos) + wordFindNext1.length();
						start = " ";
						end = ",";
						str_temp = getCoor(&pos, &start, &end, &temp);//,fout);
						x = stod(str_temp);
						str_temp = getCoor(&pos, &start, &end, &temp);//,fout);
						y = stod(str_temp);
						lane_node[cnt].coordinate.push_back({ x,y });
						num_waypoint += 1;
						
						if (x < minX) minX = x;
						if (x > maxX) maxX = x;
						if (y < minY) minY = y;
						if (y > maxY) maxY = y;
					}
					lane_node[cnt].num_waypoint = num_waypoint;
					lane_node[cnt].minX = minX;
					lane_node[cnt].minY = minY;
					lane_node[cnt].maxX = maxX;
					lane_node[cnt].maxY = maxY;
				}
				//fout << "\n";
				cnt++;
			}
		}
	}
	fin.close();
	//fout.close();
	cout << "middle save file complete" << endl;
	cout << "start connect node" << endl;
	cnt -= 1;

	//link the nodes
	int index = -1;
	for (int i = 0; i < cnt; i++)
	{
		for (int j = 0; j < cnt; j++)
		{
			
			if (!lane_node[j].fromNodeId.compare(startNodeID))
			{
				index = j;
			}
			

			if (!lane_node[i].toNodeId.compare(lane_node[j].fromNodeId))
			{
				weight.push_back({ 1, {j,i} });
				lane_node[i].linked_node_index.push_back(j);
			}

			if (!lane_node[i].rLinkId.compare(lane_node[j].ID))
			{
				weight.push_back({ 10, {j,i} });
				lane_node[i].rightLaneIndex = j;
				lane_node[j].leftLaneIndex = i;
			}

		}
	}
	if (index == -1)
		cout << "node not found" << endl;
	else
		cout << "start index : " << index << endl;
	// sort linked_node_index
	// if linked_node's lane number is smaller than others
	// sort it to the first index of the array
	for (int i = 0; i < cnt; i++)
	{
		if(lane_node[i].linked_node_index.size() > 1){
			for (int j = 0; j < (lane_node[i].linked_node_index.size() - 1); j++)
			{
				for (int k = j; k < lane_node[i].linked_node_index.size() - 1; k++)
				{
					if (lane_node[lane_node[i].linked_node_index[k + 1]].laneNum < lane_node[lane_node[i].linked_node_index[k]].laneNum)
					{
						int temp = lane_node[i].linked_node_index[k + 1];
						lane_node[i].linked_node_index[k + 1] = lane_node[i].linked_node_index[k];
						lane_node[i].linked_node_index[k] = temp;
					}
				}
			}
		}
	}
	
	ofstream fout1("final.txt");
	fout1 << fixed;
	fout1.precision(15);
	//string order = "ID \t fromnode \t tonode \t rightLaneIndex \t leftLaneIndex \t coordinate_num \t laneNum \t linked_node_num \t linked_nodes";

	fout1 << cnt << "\n";
	for (int i = 0; i < cnt; i++)
	{
		fout1 << i;
		fout1 << " ";
		fout1 << lane_node[i].ID;
		fout1 << "\t";
		fout1 << lane_node[i].fromNodeId;
		fout1 << "\t";
		fout1 << lane_node[i].toNodeId;
		fout1 << "\t";
		fout1 << lane_node[i].rightLaneIndex;
		fout1 << "\t";
		fout1 << lane_node[i].leftLaneIndex;
		fout1 << "\t";
		fout1 << lane_node[i].coordinate.size();
		fout1 << "\t";
		fout1 << lane_node[i].laneNum;
		fout1 << "\t";
		fout1 << lane_node[i].linked_node_index.size();
		fout1 << "\t";
		
		for (int j = 0; j < lane_node[i].linked_node_index.size(); j++) {
			fout1 << lane_node[i].linked_node_index[j];
			fout1 << " ";
		}
		fout1 << "\n";

		for (int k = 0; k < lane_node[i].coordinate.size(); k++)
		{
			fout1 << lane_node[i].coordinate[k][0];
			fout1 << " ";
			fout1 << lane_node[i].coordinate[k][1];
			fout1 << "\n";
		}
	}
	fout1.close();


	
	map* map_cut = new map[1000000];

	int range_x = 320000;
	int range_y = 3900000;
	int range_x1 = 540000;
	int range_y1 = 4250000;
	int diff_x = (range_x1 - range_x) / 1000;
	int diff_y = (range_y1 - range_y) / 1000;
	int small_range_x, small_range_y;
	
	for (int i = 0; i < 1000000; i++) {
		small_range_x = range_x + diff_x * (i%1000);
		small_range_y = range_y + diff_y * int(i/1000);
		map_cut[i].array = { small_range_x, small_range_y };
	}
	cout << "wait" << endl;

	vector<int> horizontal;
	vector<int> vertical;
	for (int i = 0; i < 1000; i++) {
		horizontal.push_back(320000+diff_x* i);
		vertical.push_back(3900000+diff_y* i);
	}

	ofstream fout_net("net.txt");

	cout << cnt << endl;

	for (int i = 0; i < cnt; i++) {
		unordered_map<int, int> m;
		for (int j = 0; j < lane_node[i].num_waypoint; j++) {
			int xIndex = search(lane_node[i].coordinate[j][0], horizontal);
			int yIndex = search(lane_node[i].coordinate[j][1], vertical);
			m[xIndex + 1000 * yIndex] = 1;
		}
		for (auto k = m.begin(); k != m.end(); k++)
			map_cut[k->first].struct_index.push_back(i);
	}
	

	fout_net << diff_x << " " << diff_y;
	fout_net << '\n';
	for (int i = 0; i < 1000000; i++) {
		if (map_cut[i].struct_index.size() == 0) continue;
		fout_net << map_cut[i].array.first << " " << map_cut[i].array.second << " " <<map_cut[i].struct_index.size();
		fout_net << '\n';
		for (int j = 0; j < map_cut[i].struct_index.size(); j++)
			fout_net << map_cut[i].struct_index[j] << " ";
		fout_net << '\n';
	}
	delete[] lane_node;
	delete[] map_cut;
	return 0;
}
