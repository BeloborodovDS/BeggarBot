#include "ncs_wrapper.hpp"

#include <iostream>
#include <fstream>

using namespace std;

//read whole graph file into buffer, return pointer to buffer, set filesize
void* readGraph(const char* filename, unsigned int* filesize)
{
    ifstream file;
    try
    {
        file.open(filename, ios::binary);
    }
    catch(...)
    {
        return NULL;
    }
    
    file.seekg (0, file.end);
    *filesize = file.tellg();
    file.seekg (0, file.beg);

    char* buffer = new char[*filesize];
    if (file.read(buffer, *filesize))
    {
        file.close();
        return (void*)buffer;
    }
    
    file.close();
    return NULL;
}

NCSWrapper::NCSWrapper(unsigned int input_num, unsigned int output_num, bool is_verbose)
{
    n_input = input_num;
    n_output = output_num;
    verbose = is_verbose;
    
    ncsCode = NC_OK;
    ncsDevice = NULL;
    graphSize = 0;
    graphData = NULL;
    ncsGraph = NULL;
    ncsInFifo = NULL;
    ncsOutFifo = NULL;
    resultSize = 0;
    inputSize = n_input * sizeof(float);
    resultSize = n_output * sizeof(float);
    otherParam = NULL;
    nres = 0;
    result = new float[n_output];
    
    is_init = false;
    is_allocate = false;
}

NCSWrapper::~NCSWrapper()
{
    if (result) 
        delete [] result;
    result = NULL;
    
    //deallocate graph and FIFO
    if (is_allocate)
    {
        if (ncsInFifo)
        {
            ncFifoDestroy(&ncsInFifo);
            ncsInFifo = NULL;
        }
        if (ncsOutFifo)
        {
            ncFifoDestroy(&ncsOutFifo);
            ncsOutFifo = NULL;
        }
        if (ncsGraph)
        {
            ncGraphDestroy(&ncsGraph); 
            ncsGraph = NULL;
        }
    }
    
    //free device
    if (is_init && ncsDevice)
    {
        ncDeviceClose(ncsDevice);
        ncDeviceDestroy(&ncsDevice);
        ncsDevice = NULL;
    }    
    
    otherParam = NULL;
}

bool NCSWrapper::load_file(const char* filename)
{
    //Get NCS name
    ncsCode = ncDeviceCreate(0, &ncsDevice);
    if (ncsCode != NC_OK)
    {
        if (verbose)
            cout<<"Cannot find NCS device, status: "<<ncsCode<<endl;
        return false;
    }
    if (verbose)
        cout<<"Found device No 0 "<<endl;
    
    //Open NCS device via its name
    ncsCode = ncDeviceOpen(ncsDevice);
    if (ncsCode != NC_OK)
    {  
        if (verbose)
            cout<<"Cannot open NCS device, status: "<<ncsCode<<endl;
        return false;
    }
    if (verbose)
        cout<<"Successfully opened device 0\n";
    is_init = true;
    
    //Get graph file size and data
    graphData = readGraph(filename, &graphSize);
    if (graphData==NULL)
    {
        if (verbose)
            cout<<"Cannot open graph file\n";
        return false;
    }
    if (verbose)
      cout<<"Successfully loaded graph file, size is: "<<graphSize<<endl;
    
    //Create computational graph
    ncsCode = ncGraphCreate("ncs_wrapper_graph", &ncsGraph);
    if (ncsCode != NC_OK)
    {
        if (verbose)
            cout<<"Cannot create graph, status: "<<ncsCode<<endl;
        return false;
    }
    
    //Allocate graph on NCS with input-output FIFOs
    ncsCode = ncGraphAllocateWithFifosEx(ncsDevice, ncsGraph, graphData, graphSize, 
                        &ncsInFifo, NC_FIFO_HOST_WO, 1, NC_FIFO_FP32,
                        &ncsOutFifo, NC_FIFO_HOST_RO, 1,  NC_FIFO_FP32);
    if (ncsCode != NC_OK)
    {
        if (verbose)
            cout<<"Cannot allocate graph and FIFO, status: "<<ncsCode<<endl;
        return false;
    }
    
    //raw graph data is not needed any longer
    if(graphData)
        delete [] (char*)graphData;
    graphData = NULL;
    
    if (verbose)
        cout<<"Successfully allocated graph\n";
    is_allocate = true;
    
    return true;
}


bool NCSWrapper::load_tensor(float* data, float*& output)
{    
    //load image to NCS
    ncsCode = ncGraphQueueInferenceWithFifoElem(
                ncsGraph, ncsInFifo, ncsOutFifo, (void*)data, &inputSize, NULL);
    if (ncsCode != NC_OK)
    {
        if (verbose)
            cout<<"Cannot load image to NCS, status: "<<ncsCode<<endl;
        output = NULL;
        return false;
    }
    
    //get result from NCS
    resultSize = n_output * sizeof(float);
    ncsCode = ncFifoReadElem(ncsOutFifo, (void*)result, &resultSize, &otherParam);
    if (ncsCode != NC_OK)
    {
        if (verbose)
            cout<<"Cannot retrieve result from NCS, status: "<<ncsCode<<endl;
        output = NULL;
        return false;
    }
    
    //Check result size
    nres = resultSize/sizeof(float);
    if (nres!=n_output)
    {
        if (verbose)
            cout<<"Output shape mismatch! Expected/Real: "<<n_output<<"/"<<nres<<endl;
        output = NULL;
        return false;
    }
    
    output = result;
    return true;
}

bool NCSWrapper::load_tensor_nowait(float* data)
{
    //load image to NCS
    ncsCode = ncGraphQueueInferenceWithFifoElem(
                ncsGraph, ncsInFifo, ncsOutFifo, (void*)data, &inputSize, NULL);
    if (ncsCode != NC_OK)
    {
        if (verbose)
            cout<<"Cannot load image to NCS, status: "<<ncsCode<<endl;
        return false;
    }
    return true;
}

bool NCSWrapper::get_result(float*& output)
{
    //get result from NCS
    resultSize = n_output * sizeof(float);
    ncsCode = ncFifoReadElem(ncsOutFifo, (void*)result, &resultSize, &otherParam);
    if (ncsCode != NC_OK)
    {
        if (verbose)
            cout<<"Cannot retrieve result from NCS, status: "<<ncsCode<<endl;
        output = NULL;
        return false;
    }
    
    //Check result size
    nres = resultSize/sizeof(float);
    if (nres!=n_output)
    {
        if (verbose)
            cout<<"Output shape mismatch! Expected/Real: "<<n_output<<"/"<<nres<<endl;
        output = NULL;
        return false;
    }
    
    output = result;
    return true;
}



