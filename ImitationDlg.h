
// ImitationDlg.h : ͷ�ļ�
//

#pragma once


// CImitationDlg �Ի���
class CImitationDlg : public CDialogEx
{
// ����
public:
	CImitationDlg(CWnd* pParent = NULL);	// ��׼���캯��

// �Ի�������
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_IMITATION_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV ֧��


// ʵ��
protected:
	HICON m_hIcon;

	// ���ɵ���Ϣӳ�亯��
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
