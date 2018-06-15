
// ImitationDlg.h : 头文件
//

#pragma once


// CImitationDlg 对话框
class CImitationDlg : public CDialogEx
{
// 构造
public:
	CImitationDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_IMITATION_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	afx_msg void OnBnClickedOk();
	afx_msg void OnBnClickedopenkinect();
	afx_msg void OnBnClickedopenmotor();
	afx_msg void OnBnClickedreleasekinect();
	afx_msg void OnBnClickedstarttracking();
	afx_msg void OnBnClickedclosemotor();
	afx_msg void OnBnClickedstoptracking();
	afx_msg void OnBnClickedstartimitation();
	afx_msg void OnBnClickedstopimitation();
	afx_msg void OnBnClickedmotorhome();
	afx_msg void OnBnClickedPhoto();
};
